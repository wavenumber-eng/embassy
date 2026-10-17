//! Controller register seam. Production uses typed PAC registers, never a
//! caller-provided MMIO address. Status acknowledgments are exact W1C writes.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Register {
    Istat,
    Inten,
    Errstat,
    Erren,
    Stat,
    Ctl,
    Addr,
    Bdtpage1,
    Bdtpage2,
    Bdtpage3,
    Endpoint(usize),
    Usbctrl,
    Control,
}

pub(super) trait Io {
    fn read(&mut self, register: Register) -> u8;
    fn write(&mut self, register: Register, value: u8);
    /// Orders CPU/device memory with a compiler memory clobber as well.
    fn barrier(&mut self);
}

#[cfg(target_arch = "arm")]
pub(super) struct Pac;

#[cfg(target_arch = "arm")]
impl Io for Pac {
    fn read(&mut self, register: Register) -> u8 {
        let r = crate::pac::USB0;
        match register {
            Register::Istat => r.istat().read().0,
            Register::Inten => r.inten().read().0,
            Register::Errstat => r.errstat().read().0,
            Register::Erren => r.erren().read().0,
            Register::Stat => r.stat().read().0,
            Register::Ctl => r.ctl().read().0,
            Register::Addr => r.addr().read().0,
            Register::Bdtpage1 => r.bdtpage1().read().0,
            Register::Bdtpage2 => r.bdtpage2().read().0,
            Register::Bdtpage3 => r.bdtpage3().read().0,
            Register::Endpoint(n) => r.endpoint(n).endpt().read().0,
            Register::Usbctrl => r.usbctrl().read().0,
            Register::Control => r.control().read().0,
        }
    }

    fn write(&mut self, register: Register, value: u8) {
        use crate::pac::usb::*;
        let r = crate::pac::USB0;
        match register {
            Register::Istat => r.istat().write_value(Istat(value)),
            Register::Inten => r.inten().write_value(Inten(value)),
            Register::Errstat => r.errstat().write_value(Errstat(value)),
            Register::Erren => r.erren().write_value(Erren(value)),
            Register::Ctl => r.ctl().write_value(Ctl(value)),
            Register::Addr => r.addr().write_value(Addr(value)),
            Register::Bdtpage1 => r.bdtpage1().write_value(Bdtpage1(value)),
            Register::Bdtpage2 => r.bdtpage2().write_value(Bdtpage2(value)),
            Register::Bdtpage3 => r.bdtpage3().write_value(Bdtpage3(value)),
            Register::Endpoint(n) => r.endpoint(n).endpt().write_value(Endpt(value)),
            Register::Usbctrl => r.usbctrl().write_value(Usbctrl(value)),
            Register::Control => r.control().write_value(Control(value)),
            Register::Stat => unreachable!("STAT is read-only"),
        }
    }

    fn barrier(&mut self) {
        // No nomem: the instruction must also order compiler memory accesses.
        unsafe {
            core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        }
    }
}
