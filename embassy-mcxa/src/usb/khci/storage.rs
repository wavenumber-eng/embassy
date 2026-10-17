//! Private permanently allocated DMA bounce buffers and the 512-byte BDT.
//! Derived from the reviewed eyeballs.rs KHCI storage format (MIT/Apache-2.0).

use core::cell::UnsafeCell;

pub(super) const PACKET: usize = 64;
pub(super) const SLOTS: usize = 64;
pub(super) const OWN: u32 = 1 << 7;
pub(super) const DATA1: u32 = 1 << 6;
pub(super) const DTS: u32 = 1 << 3;

#[repr(C)]
#[derive(Clone, Copy)]
struct Descriptor {
    control: u32,
    address: u32,
}

#[repr(C, align(512))]
pub(super) struct Storage {
    descriptors: UnsafeCell<[Descriptor; SLOTS]>,
    buffers: UnsafeCell<[[u8; PACKET]; SLOTS]>,
}

// SAFETY: CPU accesses are serialized by one critical section. The engine
// protocol controls OWN, barriers and reuse; no application address enters DMA.
unsafe impl Sync for Storage {}

impl Storage {
    pub const fn new() -> Self {
        Self {
            descriptors: UnsafeCell::new([Descriptor { control: 0, address: 0 }; SLOTS]),
            buffers: UnsafeCell::new([[0; PACKET]; SLOTS]),
        }
    }

    pub fn table_address(&self) -> usize {
        self.descriptors.get() as usize
    }

    pub fn buffer_address(&self, slot: usize) -> usize {
        assert!(slot < SLOTS);
        // SAFETY: Forms a raw address into this permanently allocated storage.
        unsafe { core::ptr::addr_of_mut!((*self.buffers.get())[slot]) as usize }
    }

    pub fn control(&self, slot: usize) -> u32 {
        assert!(slot < SLOTS);
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*self.descriptors.get())[slot].control)) }
    }

    pub fn set_control(&self, slot: usize, value: u32) {
        assert!(slot < SLOTS);
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!((*self.descriptors.get())[slot].control), value);
        }
    }

    pub fn set_address(&self, slot: usize) {
        let address = self.buffer_address(slot) as u32;
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!((*self.descriptors.get())[slot].address),
                address,
            );
        }
    }

    pub fn copy_in(&self, slot: usize, data: &[u8]) {
        assert!(slot < SLOTS && data.len() <= PACKET);
        let pointer = self.buffer_address(slot) as *mut u8;
        for (i, byte) in data.iter().enumerate() {
            unsafe {
                core::ptr::write_volatile(pointer.add(i), *byte);
            }
        }
    }

    pub fn copy_out(&self, slot: usize, data: &mut [u8]) {
        assert!(slot < SLOTS && data.len() <= PACKET);
        let pointer = self.buffer_address(slot) as *const u8;
        for (i, byte) in data.iter_mut().enumerate() {
            *byte = unsafe { core::ptr::read_volatile(pointer.add(i)) };
        }
    }
}

pub(super) fn slot(channel: usize, odd: bool) -> usize {
    assert!(channel < 32);
    channel * 2 + usize::from(odd)
}
