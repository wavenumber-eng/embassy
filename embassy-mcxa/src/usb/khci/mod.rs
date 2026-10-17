//! Experimental MCXA156 KHCI USBFS device driver.
//!
//! Requires the opt-in nominal FIRC48 output. This initial bench integration
//! has no VBUS sensing: the caller explicitly guarantees powered host VBUS
//! while enabled. Disable before unplugging an independently powered board.
//! Clock operating-range qualification and unrestricted hotplug are pending.
//!
//! Private DMA buffers remain allocated for the device lifetime. Only one
//! driver construction is permitted, including after drop or a fatal fault.

mod controller;
mod io;
mod storage;
mod transfer;

use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::Poll;

use controller::{Engine, Inner};
use critical_section::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_usb_driver::{
    Direction, EndpointAddress, EndpointAllocError, EndpointError, EndpointInfo, EndpointType, Event, Unsupported,
};
use io::{Io, Register as R};
use storage::{Storage, slot};
use transfer::Phase;

use crate::clocks::periph_helpers::{SPConfHelper, UsbfsConfig};
use crate::clocks::{self, ClockError, Gate, WakeGuard};
use crate::interrupt::typelevel::{Binding, Interrupt};
use crate::peripherals::USB0;
use crate::{Peri, interrupt};

static CLAIMED: AtomicBool = AtomicBool::new(false);
static STATE: Mutex<RefCell<Inner>> = Mutex::new(RefCell::new(Inner::new()));
static STORAGE: Storage = Storage::new();
static WAKERS: [AtomicWaker; 33] = [const { AtomicWaker::new() }; 33];
const BUS: usize = 32;
const POLLS: usize = 1024;

/// Experimental debug counters: reset bookkeeping, tokens, pauses and error interrupts.
#[cfg(feature = "unstable-pac")]
pub fn diagnostic_counters() -> [u32; 5] {
    with_engine(|e| {
        [
            e.state.resets,
            e.state.tokens,
            e.state.pauses,
            e.state.errors,
            e.state.error_bits as u32,
        ]
    })
}

/// Read-only bounded trace of the latest controller events for bench diagnostics.
#[cfg(feature = "unstable-pac")]
pub fn diagnostic_trace() -> ([[u32; 4]; 64], usize) {
    with_engine(|e| {
        let first = (e.state.trace_next + 64 - e.state.trace_count) % 64;
        (
            core::array::from_fn(|i| e.state.trace[(first + i) % 64]),
            e.state.trace_count,
        )
    })
}

/// Explicit bench configuration; automatic VBUS detection is not available.
#[non_exhaustive]
pub struct Config {
    /// USB0 peripheral clock configuration.
    pub clock: UsbfsConfig,
}

impl Config {
    /// Assume host VBUS is present for the entire enabled bus lifetime.
    ///
    /// For a debug-powered board, disable USB before unplugging native USB.
    /// This does not claim VBUS detection or production self-powered hotplug.
    pub fn assume_vbus_present() -> Self {
        Self {
            clock: UsbfsConfig::default(),
        }
    }
}

/// USB construction error.
#[derive(Debug)]
pub enum InitError {
    /// Static controller state was already claimed, possibly by a dropped driver.
    AlreadyTaken,
    /// Clock preparation failed.
    Clock(ClockError),
    /// Private DMA allocation is outside the reviewed normal SRAM region.
    Storage,
    /// A bounded MRCC state readback failed.
    ResetTimeout,
}

fn with_engine<T>(f: impl FnOnce(&mut Engine<'_, io::Pac>) -> T) -> T {
    let (result, notify) = critical_section::with(|cs| {
        let mut state = STATE.borrow_ref_mut(cs);
        let before = state.notifications;
        let result = f(&mut Engine {
            io: io::Pac,
            state: &mut state,
            storage: &STORAGE,
        });
        (result, before != state.notifications)
    });
    if notify {
        wake_all();
    }
    result
}

fn wake_all() {
    for w in &WAKERS {
        w.wake();
    }
}

fn bounded(mut condition: impl FnMut() -> bool) -> Result<(), InitError> {
    for _ in 0..POLLS {
        if condition() {
            return Ok(());
        }
    }
    Err(InitError::ResetTimeout)
}

fn prepare_gate(clock: &UsbfsConfig) -> Result<Option<WakeGuard>, InitError> {
    critical_section::with(|_| {
        interrupt::typelevel::USB0::disable();
        if !clocks::usbfs::set_reset_held(true) {
            return Err(InitError::ResetTimeout);
        }
        unsafe {
            USB0::disable_clock();
        }
        bounded(|| !USB0::is_clock_enabled())?;
        let parts = clocks::with_clocks(|clocks| clock.pre_enable_config(clocks))
            .ok_or(InitError::Clock(ClockError::NeverInitialized))?
            .map_err(InitError::Clock)?;
        unsafe {
            USB0::enable_clock();
        }
        bounded(USB0::is_clock_enabled)?;
        io::Pac.barrier();
        Ok(parts.wake_guard)
    })
}

/// USB0 interrupt binding, paired permanently with this driver's static state.
pub struct InterruptHandler;

impl interrupt::typelevel::Handler<interrupt::typelevel::USB0> for InterruptHandler {
    unsafe fn on_interrupt() {
        let result = with_engine(|e| e.interrupt());
        wake_all();
        assert!(result.is_ok(), "KHCI controller fault: {:?}", result);
    }
}

/// Native USBFS device driver.
pub struct Driver<'d> {
    bus: Bus<'d>,
    allocated: u16,
}

impl<'d> Driver<'d> {
    /// Own USB0, its interrupt binding and its unique DMA storage association.
    pub fn new(
        usb: Peri<'d, USB0>,
        _irq: impl Binding<interrupt::typelevel::USB0, InterruptHandler> + 'd,
        config: Config,
    ) -> Result<Self, InitError> {
        CLAIMED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| InitError::AlreadyTaken)?;
        let begin = STORAGE.table_address();
        let end = begin
            .checked_add(core::mem::size_of::<Storage>())
            .ok_or(InitError::Storage)?;
        if begin & 511 != 0 || begin < 0x2000_0000 || end > 0x2001_e000 {
            return Err(InitError::Storage);
        }
        let guard = prepare_gate(&config.clock)?;
        Ok(Self {
            bus: Bus {
                _usb: usb,
                clock: config.clock,
                guard,
                clocked: true,
            },
            allocated: 1,
        })
    }

    fn allocate(
        &mut self,
        direction: Direction,
        ep_type: EndpointType,
        requested: Option<EndpointAddress>,
        size: u16,
        interval_ms: u8,
    ) -> Result<Endpoint<'d>, EndpointAllocError> {
        let valid = match ep_type {
            EndpointType::Bulk => matches!(size, 8 | 16 | 32 | 64),
            EndpointType::Interrupt => (1..=64).contains(&size) && interval_ms != 0,
            _ => false,
        };
        if !valid || requested.is_some_and(|a| a.direction() != direction) {
            return Err(EndpointAllocError);
        }
        // Reserve the endpoint number in both directions. This initial backend
        // avoids the hardware's shared per-number EPSTALL coupling.
        let n = requested
            .map(|a| a.index())
            .unwrap_or_else(|| (1..16).find(|n| self.allocated & (1 << n) == 0).unwrap_or(16));
        if n == 0 || n >= 16 || self.allocated & (1 << n) != 0 {
            return Err(EndpointAllocError);
        }
        self.allocated |= 1 << n;
        Ok(Endpoint {
            info: EndpointInfo {
                addr: EndpointAddress::from_parts(n, direction),
                ep_type,
                max_packet_size: size,
                interval_ms,
            },
            _lifetime: PhantomData,
        })
    }
}

impl<'d> embassy_usb_driver::Driver<'d> for Driver<'d> {
    type EndpointOut = Endpoint<'d>;
    type EndpointIn = Endpoint<'d>;
    type ControlPipe = ControlPipe<'d>;
    type Bus = Bus<'d>;
    fn alloc_endpoint_out(
        &mut self,
        t: EndpointType,
        a: Option<EndpointAddress>,
        s: u16,
        i: u8,
    ) -> Result<Self::EndpointOut, EndpointAllocError> {
        self.allocate(Direction::Out, t, a, s, i)
    }
    fn alloc_endpoint_in(
        &mut self,
        t: EndpointType,
        a: Option<EndpointAddress>,
        s: u16,
        i: u8,
    ) -> Result<Self::EndpointIn, EndpointAllocError> {
        self.allocate(Direction::In, t, a, s, i)
    }
    fn start(self, control_max_packet_size: u16) -> (Self::Bus, Self::ControlPipe) {
        assert!(matches!(control_max_packet_size, 8 | 16 | 32 | 64));
        (
            self.bus,
            ControlPipe {
                max_packet_size: control_max_packet_size as usize,
                generation: 0,
                _lifetime: PhantomData,
            },
        )
    }
}

/// USB bus control, retaining peripheral and clock ownership.
pub struct Bus<'d> {
    _usb: Peri<'d, USB0>,
    clock: UsbfsConfig,
    guard: Option<WakeGuard>,
    clocked: bool,
}

impl Bus<'_> {
    fn shutdown(&mut self) {
        interrupt::typelevel::USB0::disable();
        if !self.clocked {
            return;
        }
        critical_section::with(|cs| {
            let mut state = STATE.borrow_ref_mut(cs);
            if state.active {
                let mut io = io::Pac;
                io.write(R::Control, 0);
                io.write(R::Inten, 0);
                io.write(R::Ctl, 0);
                io.barrier();
            }
            state.active = false;
            for c in &mut state.channels {
                c.enabled = false;
            }
            assert!(clocks::usbfs::set_reset_held(true), "USB reset assertion timed out");
            io::Pac.barrier();
            unsafe {
                USB0::disable_clock();
            }
            bounded(|| !USB0::is_clock_enabled()).expect("USB clock disable timed out");
            io::Pac.barrier();
            interrupt::typelevel::USB0::unpend();
        });
        self.clocked = false;
        self.guard = None;
        wake_all();
    }
}

impl Drop for Bus<'_> {
    fn drop(&mut self) {
        self.shutdown();
    }
}

impl embassy_usb_driver::Bus for Bus<'_> {
    async fn enable(&mut self) {
        if with_engine(|e| e.state.active) {
            return;
        }
        if !self.clocked {
            self.guard = prepare_gate(&self.clock).expect("USB clock restart failed");
            self.clocked = true;
        }
        assert!(clocks::usbfs::set_reset_held(false), "USB reset release timed out");
        io::Pac.barrier();
        with_engine(|e| e.initialize());
        interrupt::typelevel::USB0::unpend();
        unsafe {
            interrupt::typelevel::USB0::enable();
        }
    }
    async fn disable(&mut self) {
        self.shutdown();
    }
    async fn poll(&mut self) -> Event {
        poll_fn(|cx| {
            WAKERS[BUS].register(cx.waker());
            with_engine(|e| e.state.next_event().map_or(Poll::Pending, Poll::Ready))
        })
        .await
    }
    fn endpoint_set_enabled(&mut self, address: EndpointAddress, enabled: bool) {
        let index = address.index() * 2 + usize::from(address.is_in());
        assert!(index < 32 && address.index() != 0);
        let result = with_engine(|e| e.endpoint(index, enabled, false));
        wake_all();
        assert!(result.is_ok(), "USB endpoint transition failed");
    }
    fn endpoint_set_stalled(&mut self, address: EndpointAddress, stalled: bool) {
        let index = address.index() * 2 + usize::from(address.is_in());
        assert!(index < 32 && address.index() != 0);
        let result = with_engine(|e| {
            let enabled = e.state.channels[index].enabled;
            e.endpoint(index, enabled, stalled)
        });
        wake_all();
        assert!(result.is_ok(), "USB halt transition failed");
    }
    fn endpoint_is_stalled(&mut self, address: EndpointAddress) -> bool {
        let index = address.index() * 2 + usize::from(address.is_in());
        with_engine(|e| e.state.channels[index].stalled)
    }
    fn force_reset(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
    async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
        Err(Unsupported)
    }
}

struct TransferGuard {
    index: usize,
    id: u64,
}
impl Drop for TransferGuard {
    fn drop(&mut self) {
        with_engine(|e| e.state.channels[self.index].abandon(self.id));
    }
}

/// One allocated non-control endpoint. Opposite directions use distinct numbers.
pub struct Endpoint<'d> {
    info: EndpointInfo,
    _lifetime: PhantomData<&'d mut USB0>,
}

impl Endpoint<'_> {
    fn index(&self) -> usize {
        self.info.addr.index() * 2 + usize::from(self.info.addr.is_in())
    }
    async fn transfer(&mut self, write: Option<&[u8]>, read: Option<&mut [u8]>) -> Result<usize, EndpointError> {
        let index = self.index();
        let ticket = poll_fn(|cx| {
            WAKERS[index].register(cx.waker());
            with_engine(|e| {
                if !e.state.active || !e.state.channels[index].enabled {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                let length = write.map_or(self.info.max_packet_size as usize, |d| d.len());
                match e.begin(index, length, write, None) {
                    Some(ticket) => Poll::Ready(Ok(ticket)),
                    None if !e.state.active || !e.state.channels[index].enabled => {
                        Poll::Ready(Err(EndpointError::Disabled))
                    }
                    None => Poll::Pending,
                }
            })
        })
        .await?;
        let _guard = TransferGuard { index, id: ticket.0 };
        let mut read = read;
        poll_fn(|cx| {
            WAKERS[index].register(cx.waker());
            with_engine(|e| {
                let c = e.state.channels[index];
                if !e.state.active || !c.enabled || c.generation != ticket.1 {
                    return Poll::Ready(Err(EndpointError::Disabled));
                }
                if let Phase::Complete { id, length } = c.phase {
                    if id == ticket.0 {
                        let result = if let Some(data) = read.as_deref_mut() {
                            if length > data.len() {
                                Err(EndpointError::BufferOverflow)
                            } else {
                                e.storage.copy_out(slot(index, c.completed_bank), &mut data[..length]);
                                Ok(length)
                            }
                        } else {
                            Ok(length)
                        };
                        e.state.channels[index].phase = Phase::Idle;
                        return Poll::Ready(result);
                    }
                }
                Poll::Pending
            })
        })
        .await
    }
}

impl embassy_usb_driver::Endpoint for Endpoint<'_> {
    fn info(&self) -> &EndpointInfo {
        &self.info
    }
    async fn wait_enabled(&mut self) {
        let index = self.index();
        poll_fn(|cx| {
            WAKERS[index].register(cx.waker());
            with_engine(|e| {
                if e.state.active && e.state.channels[index].enabled {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
        })
        .await
    }
}
impl embassy_usb_driver::EndpointIn for Endpoint<'_> {
    async fn write(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        if data.len() > self.info.max_packet_size as usize {
            return Err(EndpointError::BufferOverflow);
        }
        self.transfer(Some(data), None).await.map(|_| ())
    }
}
impl embassy_usb_driver::EndpointOut for Endpoint<'_> {
    async fn read(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.transfer(None, Some(data)).await
    }
}

/// Endpoint-zero control pipe with SETUP generation preemption.
pub struct ControlPipe<'d> {
    max_packet_size: usize,
    generation: u64,
    _lifetime: PhantomData<&'d mut USB0>,
}

impl ControlPipe<'_> {
    async fn send(&mut self, data: &[u8], last: bool) -> Result<(), EndpointError> {
        if data.len() > self.max_packet_size {
            return Err(EndpointError::BufferOverflow);
        }
        let generation = self.generation;
        let (id, _) =
            with_engine(|e| e.begin(1, data.len(), Some(data), Some(generation))).ok_or(EndpointError::Disabled)?;
        let _guard = TransferGuard { index: 1, id };
        let mut done = false;
        poll_fn(|cx| {
            WAKERS[0].register(cx.waker());
            WAKERS[1].register(cx.waker());
            with_engine(|e| match e.control_in_poll(id, generation, last, &mut done) {
                Ok(Some(())) => Poll::Ready(Ok(())),
                Ok(None) => Poll::Pending,
                Err(error) => Poll::Ready(Err(error)),
            })
        })
        .await
    }
}

impl embassy_usb_driver::ControlPipe for ControlPipe<'_> {
    fn max_packet_size(&self) -> usize {
        self.max_packet_size
    }
    async fn setup(&mut self) -> [u8; 8] {
        let (generation, setup) = poll_fn(|cx| {
            WAKERS[0].register(cx.waker());
            WAKERS[1].register(cx.waker());
            with_engine(|e| e.setup_poll().map_or(Poll::Pending, Poll::Ready))
        })
        .await;
        self.generation = generation;
        setup
    }
    async fn data_out(&mut self, data: &mut [u8], _first: bool, _last: bool) -> Result<usize, EndpointError> {
        let generation = self.generation;
        poll_fn(|cx| {
            WAKERS[0].register(cx.waker());
            WAKERS[1].register(cx.waker());
            with_engine(|e| match e.control_out_poll(generation, data) {
                Ok(Some(length)) => Poll::Ready(Ok(length)),
                Ok(None) => Poll::Pending,
                Err(error) => Poll::Ready(Err(error)),
            })
        })
        .await
    }
    async fn data_in(&mut self, data: &[u8], _first: bool, last: bool) -> Result<(), EndpointError> {
        self.send(data, last).await
    }
    async fn accept(&mut self) {
        let _ = self.send(&[], false).await;
    }
    async fn reject(&mut self) {
        let generation = self.generation;
        let result = with_engine(|e| e.reject(generation));
        assert!(result.is_ok(), "USB rejection transition failed");
        poll_fn(|cx| {
            WAKERS[0].register(cx.waker());
            WAKERS[1].register(cx.waker());
            with_engine(|e| {
                if !e.state.active || e.state.setup_generation != generation {
                    Poll::Ready(())
                } else {
                    e.discard_out();
                    Poll::Pending
                }
            })
        })
        .await
    }
    async fn accept_set_address(&mut self, address: u8) {
        if self.send(&[], false).await.is_ok() {
            let result = with_engine(|e| e.set_address(self.generation, address));
            assert!(result.is_ok(), "USB address transition failed");
        }
    }
}
