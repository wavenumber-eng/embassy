//! Private KHCI engine. The wrapper holds one critical section across each
//! operation, including DMA preparation and IRQ/FIFO handling.

use embassy_usb_driver::{EndpointError, Event};

use super::io::{Io, Register as R};
use super::storage::{DATA1, DTS, OWN, PACKET, SLOTS, Storage, slot};
use super::transfer::{Channel, Phase};

pub(super) const RESET: u8 = 1;
pub(super) const ERROR: u8 = 2;
pub(super) const TOKEN: u8 = 8;
pub(super) const SLEEP: u8 = 16;
pub(super) const RESUME: u8 = 32;
pub(super) const STALL: u8 = 128;
pub(super) const POWER_EVENT: u8 = 1;
pub(super) const RESET_EVENT: u8 = 2;
const ENABLE: u8 = 1;
const ODDRST: u8 = 2;
const TOKEN_BUSY: u8 = 32;
const PULLUP: u8 = 16;
#[cfg(test)]
const ENDPOINT_ENABLES: u8 = 4 | 8;
const POLLS: usize = 1024;
const DRAIN: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Fault {
    StopTimeout,
    DrainTimeout,
    Ownership,
    Length,
}

pub(super) struct Inner {
    pub active: bool,
    pub events: u8,
    pub channels: [Channel; 32],
    pub next_id: u64,
    pub setup_generation: u64,
    pub setup_pending: bool,
    pub setup: [u8; 8],
    pub out_packet: Option<(u64, usize)>,
    pub out_data: [u8; PACKET],
    pub fault: Option<Fault>,
    pub resets: u32,
    pub tokens: u32,
    pub pauses: u32,
    pub errors: u32,
    pub error_bits: u8,
    pub notifications: u64,
    pub suspended: bool,
    pub reported_suspended: bool,
    pub reset_ack_pending: bool,
    pub paused_endpoints: Option<[u8; 16]>,
    pub pause_mask: [u8; 16],
    pub trace: [[u32; 4]; 64],
    pub trace_count: usize,
    pub trace_next: usize,
}

impl Inner {
    pub const fn new() -> Self {
        Self {
            active: false,
            events: POWER_EVENT,
            channels: [Channel::new(); 32],
            next_id: 1,
            setup_generation: 0,
            setup_pending: false,
            setup: [0; 8],
            out_packet: None,
            out_data: [0; PACKET],
            fault: None,
            resets: 0,
            tokens: 0,
            pauses: 0,
            errors: 0,
            error_bits: 0,
            notifications: 0,
            suspended: false,
            reported_suspended: false,
            reset_ack_pending: false,
            paused_endpoints: None,
            pause_mask: [0; 16],
            trace: [[0; 4]; 64],
            trace_count: 0,
            trace_next: 0,
        }
    }

    pub fn record(&mut self, event: [u32; 4]) {
        self.trace[self.trace_next] = event;
        self.trace_next = (self.trace_next + 1) % self.trace.len();
        self.trace_count = (self.trace_count + 1).min(self.trace.len());
    }

    pub fn id(&mut self) -> u64 {
        let id = self.next_id;
        self.next_id = id.checked_add(1).expect("USB transfer IDs exhausted");
        id
    }

    fn notify(&mut self) {
        self.notifications = self
            .notifications
            .checked_add(1)
            .expect("USB notification IDs exhausted");
    }

    pub fn next_event(&mut self) -> Option<Event> {
        for (bit, event) in [(POWER_EVENT, Event::PowerDetected), (RESET_EVENT, Event::Reset)] {
            if self.events & bit != 0 {
                self.events &= !bit;
                return Some(event);
            }
        }
        if self.suspended != self.reported_suspended {
            self.reported_suspended = self.suspended;
            return Some(if self.suspended { Event::Suspend } else { Event::Resume });
        }
        None
    }
}

pub(super) struct Engine<'a, B> {
    pub io: B,
    pub state: &'a mut Inner,
    pub storage: &'a Storage,
}

impl<B: Io> Engine<'_, B> {
    pub fn fail(&mut self, fault: Fault) -> Fault {
        self.state.notify();
        self.io.write(R::Inten, 0);
        let control = self.io.read(R::Control);
        self.io.write(R::Control, control & !PULLUP);
        let ctl = self.io.read(R::Ctl);
        self.io.write(R::Ctl, ctl & !ENABLE);
        self.io.barrier();
        self.state.active = false;
        self.state.fault = Some(fault);
        for c in &mut self.state.channels {
            c.enabled = false;
        }
        // Do NOT reclaim descriptors or release the permanent storage here.
        fault
    }

    /// Disable endpoint token acceptance without disabling USB reset detection.
    /// The readback/drain boundary is an explicit bench engineering inference,
    /// not a documented DMA-idle handshake. Storage remains permanently private.
    #[cfg(test)]
    pub fn pause(&mut self) -> Result<(), Fault> {
        self.pause_with(ENDPOINT_ENABLES)
    }

    fn pause_rx(&mut self) -> Result<(), Fault> {
        self.pause_with(8)
    }

    fn pause_with(&mut self, mask: u8) -> Result<(), Fault> {
        if self.state.paused_endpoints.is_none() {
            let mut endpoints = [0; 16];
            for (n, saved) in endpoints.iter_mut().enumerate() {
                *saved = self.io.read(R::Endpoint(n));
            }
            self.state.paused_endpoints = Some(endpoints);
        }
        for stop in &mut self.state.pause_mask {
            *stop |= mask;
        }
        let masks = self.state.pause_mask;
        let endpoints = self.state.paused_endpoints.unwrap();
        for (n, value) in endpoints.iter().enumerate() {
            self.io.write(R::Endpoint(n), *value & !masks[n]);
        }
        for (n, mask) in masks.iter().enumerate() {
            let mut stopped = false;
            for _ in 0..POLLS {
                if self.io.read(R::Endpoint(n)) & mask == 0 {
                    stopped = true;
                    break;
                }
            }
            if !stopped {
                return Err(self.fail(Fault::StopTimeout));
            }
        }
        self.io.barrier();
        self.state.pauses = self.state.pauses.saturating_add(1);
        Ok(())
    }

    fn pause_tx(&mut self, n: usize) -> Result<(), Fault> {
        self.state.pause_mask[n] |= 4;
        let value = self.read_endpoint(n) & !self.state.pause_mask[n];
        self.io.write(R::Endpoint(n), value);
        for _ in 0..POLLS {
            if self.io.read(R::Endpoint(n)) & 4 == 0 {
                self.io.barrier();
                return Ok(());
            }
        }
        Err(self.fail(Fault::StopTimeout))
    }

    fn read_endpoint(&mut self, n: usize) -> u8 {
        match &self.state.paused_endpoints {
            Some(values) => values[n],
            None => self.io.read(R::Endpoint(n)),
        }
    }

    fn write_endpoint(&mut self, n: usize, value: u8) {
        if let Some(values) = &mut self.state.paused_endpoints {
            values[n] = value;
            self.io.write(R::Endpoint(n), value & !self.state.pause_mask[n]);
        } else {
            self.io.write(R::Endpoint(n), value);
        }
    }

    fn resume(&mut self) {
        if !self.state.active || self.state.fault.is_some() {
            return;
        }
        self.io.barrier();
        let ctl = self.io.read(R::Ctl);
        // Clear the SETUP token-busy latch BEFORE restoring endpoint acceptance.
        // A newer SETUP arriving after restoration must retain its own latch.
        self.io.write(R::Ctl, (ctl | ENABLE) & !TOKEN_BUSY);
        self.io.barrier();
        if self.state.reset_ack_pending {
            self.io.write(R::Istat, RESET);
            self.io.barrier();
            self.state.reset_ack_pending = false;
        }
        if let Some(endpoints) = self.state.paused_endpoints.take() {
            self.state.pause_mask = [0; 16];
            for (n, value) in endpoints.iter().enumerate().skip(1) {
                self.io.write(R::Endpoint(n), *value);
            }
            self.io.barrier();
            self.io.write(R::Endpoint(0), endpoints[0]);
            self.io.barrier();
        }
    }

    /// Engine is paused or the peripheral is freshly reset, and the FIFO is empty.
    pub fn reset_paused(&mut self) {
        self.state.record([1, self.state.tokens, 0, 0]);
        self.state.notify();
        self.state.suspended = false;
        self.state.reported_suspended = false;
        self.io.write(R::Istat, SLEEP | RESUME | STALL | ERROR);
        self.state.setup_generation = self.state.setup_generation.checked_add(1).expect("USB epoch exhausted");
        self.state.setup_pending = false;
        self.state.out_packet = None;
        self.io.write(R::Addr, 0);
        let ctl = self.io.read(R::Ctl) & !(ODDRST | TOKEN_BUSY);
        self.io.write(R::Ctl, ctl | ODDRST);
        for s in 0..SLOTS {
            self.storage.set_control(s, 0);
            self.storage.set_address(s);
        }
        for (i, c) in self.state.channels.iter_mut().enumerate() {
            c.reclaim_quiesced(true);
            c.enabled = i < 2;
        }
        for n in 0..16 {
            self.write_endpoint(n, 0);
        }
        self.io.write(R::Ctl, ctl);
        self.io.write(R::Errstat, 0xff);
        self.io.write(R::Erren, 0xff);
        self.write_endpoint(0, 1 | 4 | 8);
        self.prime_out();
        self.state.resets = self.state.resets.saturating_add(1);
    }

    pub fn initialize(&mut self) {
        assert!(self.state.fault.is_none(), "faulted USB controller cannot be reused");
        self.io.write(R::Inten, 0);
        self.io.write(R::Control, 0);
        self.io.write(R::Usbctrl, 0);
        self.io.write(R::Ctl, 0);
        self.io.write(R::Istat, 0xff);
        let address = self.storage.table_address();
        assert_eq!(address & 511, 0);
        self.io.write(R::Bdtpage1, ((address >> 8) & 0xfe) as u8);
        self.io.write(R::Bdtpage2, (address >> 16) as u8);
        self.io.write(R::Bdtpage3, (address >> 24) as u8);
        self.state.active = true;
        self.reset_paused();
        self.io.write(R::Inten, RESET | ERROR | TOKEN | SLEEP | RESUME | STALL);
        self.resume();
        self.io.write(R::Control, PULLUP);
        self.io.barrier();
    }

    fn publish(&mut self, index: usize, length: usize, data: Option<&[u8]>) -> Option<(u64, u64)> {
        if !self.state.active || self.state.fault.is_some() || length > PACKET {
            return None;
        }
        let id = self.state.id();
        let channel = &mut self.state.channels[index];
        let generation = channel.reserve(id)?;
        let s = slot(index, channel.odd);
        if self.storage.control(s) & OWN != 0 {
            self.fail(Fault::Ownership);
            return None;
        }
        if let Some(bytes) = data {
            self.storage.copy_in(s, bytes);
        }
        self.storage.set_address(s);
        if !channel.publish(id, generation) {
            return None;
        }
        let control = ((length as u32) << 16) | DTS | if channel.data1 { DATA1 } else { 0 };
        self.io.barrier();
        self.storage.set_control(s, control | OWN);
        self.state.record([3, index as u32, s as u32, control | OWN]);
        self.io.barrier();
        Some((id, generation))
    }

    pub fn begin(
        &mut self,
        index: usize,
        length: usize,
        data: Option<&[u8]>,
        setup: Option<u64>,
    ) -> Option<(u64, u64)> {
        if !self.state.active || self.state.fault.is_some() {
            return None;
        }
        self.pause_rx().ok()?;
        self.account_paused().ok()?;
        let result = if setup.is_some_and(|g| g != self.state.setup_generation) {
            None
        } else {
            self.publish(index, length, data)
        };
        self.resume();
        result
    }

    pub fn prime_out(&mut self) {
        if self.state.out_packet.is_none() && self.state.channels[0].phase == Phase::Idle {
            let _ = self.publish(0, PACKET, None);
        }
    }

    pub fn setup_poll(&mut self) -> Option<(u64, [u8; 8])> {
        if self.state.setup_pending {
            self.state.setup_pending = false;
            return Some((self.state.setup_generation, self.state.setup));
        }
        // A cancelled/aborted request must not keep the only OUT bank idle
        // while setup() waits forever for a packet it cannot receive.
        self.discard_out();
        None
    }

    pub fn discard_out(&mut self) {
        self.state.out_packet = None;
        self.prime_out();
    }

    pub fn control_out_poll(&mut self, generation: u64, data: &mut [u8]) -> Result<Option<usize>, EndpointError> {
        if !self.state.active || self.state.setup_generation != generation {
            return Err(EndpointError::Disabled);
        }
        let Some((packet_generation, length)) = self.state.out_packet else {
            self.prime_out();
            return Ok(None);
        };
        if packet_generation != generation {
            return Err(EndpointError::Disabled);
        }
        let result = if length <= data.len() {
            data[..length].copy_from_slice(&self.state.out_data[..length]);
            Ok(Some(length))
        } else {
            Err(EndpointError::BufferOverflow)
        };
        self.discard_out();
        result
    }

    pub fn control_in_poll(
        &mut self,
        id: u64,
        generation: u64,
        last: bool,
        done: &mut bool,
    ) -> Result<Option<()>, EndpointError> {
        if !self.state.active || self.state.setup_generation != generation {
            return Err(EndpointError::Disabled);
        }
        if let Phase::Complete { id: current, .. } = self.state.channels[1].phase
            && current == id
        {
            self.state.channels[1].phase = Phase::Idle;
            *done = true;
        }
        if self.state.out_packet.is_some() && (!*done || !last) {
            // Legal early OUT status can terminate a control read before the
            // pending IN is ACKed. Re-prime OUT for a newer SETUP immediately.
            self.state.channels[1].abandon(id);
            self.discard_out();
            return Err(EndpointError::Disabled);
        }
        if !*done {
            return Ok(None);
        }
        if !last {
            return Ok(Some(()));
        }
        if let Some((packet_generation, length)) = self.state.out_packet {
            self.discard_out();
            if packet_generation == generation && length == 0 {
                Ok(Some(()))
            } else {
                Err(EndpointError::Disabled)
            }
        } else {
            Ok(None)
        }
    }

    /// Snapshots descriptor/payload BEFORE exact TOKDNE W1C advances the FIFO.
    /// No descriptor is rearmed during this drain, including EP0.
    pub fn drain(&mut self) -> Result<bool, Fault> {
        let mut setup = false;
        for _ in 0..DRAIN {
            if self.io.read(R::Istat) & TOKEN == 0 {
                return Ok(setup);
            }
            let stat = self.io.read(R::Stat);
            let index = usize::from(stat >> 4) * 2 + usize::from(stat & 8 != 0);
            let odd = stat & 4 != 0;
            let s = slot(index, odd);
            let control = self.storage.control(s);
            self.io.barrier();
            let length = ((control >> 16) & 0x3ff) as usize;
            if control & OWN != 0 {
                return Err(self.fail(Fault::Ownership));
            }
            if length > PACKET {
                return Err(self.fail(Fault::Length));
            }
            self.state
                .record([2, stat as u32, control, self.io.read(R::Ctl) as u32]);
            self.state.tokens = self.state.tokens.saturating_add(1);
            self.state.notify();
            if index == 0 {
                if !self.state.channels[0].complete(odd, length) {
                    return Err(self.fail(Fault::Ownership));
                }
                if (control >> 2) & 15 == 13 {
                    if length != 8 {
                        return Err(self.fail(Fault::Length));
                    }
                    self.storage.copy_out(s, &mut self.state.setup);
                    self.state.record([
                        4,
                        u32::from_le_bytes(self.state.setup[..4].try_into().unwrap()),
                        u32::from_le_bytes(self.state.setup[4..].try_into().unwrap()),
                        0,
                    ]);
                    self.state.setup_generation =
                        self.state.setup_generation.checked_add(1).expect("USB epoch exhausted");
                    self.state.setup_pending = true;
                    self.state.out_packet = None;
                    setup = true;
                } else {
                    self.storage.copy_out(s, &mut self.state.out_data[..length]);
                    self.state.out_packet = Some((self.state.setup_generation, length));
                }
                self.state.channels[0].phase = Phase::Idle;
            } else if !self.state.channels[index].complete(odd, length) {
                return Err(self.fail(Fault::Ownership));
            }
            self.io.write(R::Istat, TOKEN);
        }
        Err(self.fail(Fault::DrainTimeout))
    }

    fn finish_setup_paused(&mut self) -> Result<(), Fault> {
        // Device SETUP automatically holds token processing. Keep that latch
        // until obsolete EP0 IN ownership is reclaimed and OUT is re-primed.
        if self.io.read(R::Ctl) & TOKEN_BUSY == 0 {
            return Err(self.fail(Fault::Ownership));
        }
        for i in 0..2 {
            self.state.channels[i].reclaim_quiesced(false);
            self.state.channels[i].enabled = true;
            self.state.channels[i].data1 = true;
            for odd in [false, true] {
                self.storage.set_control(slot(i, odd), 0);
            }
        }
        self.state.out_packet = None;
        self.write_endpoint(0, 1 | 4 | 8);
        self.prime_out();
        Ok(())
    }

    fn account_paused(&mut self) -> Result<(), Fault> {
        let setup = self.drain()?;
        if self.io.read(R::Istat) & RESET != 0 {
            self.state.reset_ack_pending = true;
            self.reset_paused();
            self.state.events |= RESET_EVENT;
        } else if setup {
            self.finish_setup_paused()?;
        }
        Ok(())
    }

    pub fn reject(&mut self, generation: u64) -> Result<(), Fault> {
        if let Some(fault) = self.state.fault {
            return Err(fault);
        }
        if !self.state.active {
            return Ok(());
        }
        self.pause_rx()?;
        self.account_paused()?;
        if self.state.setup_generation == generation {
            self.discard_out();
            let value = self.read_endpoint(0);
            self.write_endpoint(0, value | 2);
        }
        self.resume();
        Ok(())
    }

    pub fn interrupt(&mut self) -> Result<(), Fault> {
        if !self.state.active {
            return Ok(());
        }
        let pending = self.io.read(R::Istat) & self.io.read(R::Inten);
        let mut reset = false;
        if pending & (RESET | TOKEN) != 0 {
            self.pause_rx()?;
            let setup = self.drain()?;
            if self.io.read(R::Istat) & RESET != 0 {
                self.state.reset_ack_pending = true;
                self.reset_paused();
                self.state.events |= RESET_EVENT;
                reset = true;
            } else if setup {
                self.finish_setup_paused()?;
            }
            self.resume();
        }
        if !reset && pending & SLEEP != 0 {
            self.io.write(R::Istat, SLEEP);
            self.state.suspended = true;
            self.state.notify();
        }
        if !reset && pending & RESUME != 0 {
            self.io.write(R::Istat, RESUME);
            self.state.suspended = false;
            self.state.notify();
        }
        if pending & ERROR != 0 {
            self.state.errors = self.state.errors.saturating_add(1);
            let errors = self.io.read(R::Errstat);
            self.state.error_bits |= errors;
            self.io.write(R::Errstat, errors);
            self.io.write(R::Istat, ERROR);
        }
        if pending & STALL != 0 {
            self.io.write(R::Istat, STALL);
        }
        Ok(())
    }

    /// Pause/drain preserves other endpoints and their completion records.
    pub fn endpoint(&mut self, index: usize, enabled: bool, stalled: bool) -> Result<(), Fault> {
        if !self.state.active {
            return Ok(());
        }
        self.pause_rx()?;
        self.state.notify();
        let mut setup = self.drain()?;
        if index % 2 == 1 && matches!(self.state.channels[index].phase, Phase::Published { .. }) {
            // Only this IN endpoint needs stronger suspension to reclaim OWN.
            // Preserve unrelated transmit acceptance, including EP0 status.
            self.pause_tx(index / 2)?;
            setup |= self.drain()?;
        }
        if self.io.read(R::Istat) & RESET != 0 {
            self.state.reset_ack_pending = true;
            self.reset_paused();
            self.state.events |= RESET_EVENT;
        } else {
            if setup {
                self.finish_setup_paused()?;
            }
            let channel = &mut self.state.channels[index];
            channel.reclaim_quiesced(false);
            channel.enabled = enabled;
            channel.stalled = stalled;
            for odd in [false, true] {
                self.storage.set_control(slot(index, odd), 0);
            }
            let n = index / 2;
            let out = self.state.channels[n * 2];
            let input = self.state.channels[n * 2 + 1];
            let value = 1
                | if n != 0 { 16 } else { 0 }
                | if out.enabled { 8 } else { 0 }
                | if input.enabled { 4 } else { 0 }
                | if out.stalled || input.stalled { 2 } else { 0 };
            self.write_endpoint(n, value);
        }
        self.resume();
        Ok(())
    }

    pub fn set_address(&mut self, generation: u64, address: u8) -> Result<(), Fault> {
        if let Some(fault) = self.state.fault {
            return Err(fault);
        }
        if !self.state.active {
            return Ok(());
        }
        self.pause_rx()?;
        let setup = self.drain()?;
        if self.io.read(R::Istat) & RESET != 0 {
            self.state.reset_ack_pending = true;
            self.reset_paused();
            self.state.events |= RESET_EVENT;
        } else if setup {
            self.finish_setup_paused()?;
        } else if self.state.setup_generation == generation {
            self.io.write(R::Addr, address & 127);
        }
        self.resume();
        Ok(())
    }
}
