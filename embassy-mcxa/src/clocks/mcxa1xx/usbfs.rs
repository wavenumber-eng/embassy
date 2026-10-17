//! One-write USB0 reset requests with a bounded readback protocol. The general
//! Gate reset helpers contain unbounded waits and are not used by USB shutdown.

pub(crate) trait ResetRegisters {
    fn request(&mut self, held: bool);
    fn released(&mut self) -> bool;
}

pub(crate) fn request_reset(regs: &mut impl ResetRegisters, held: bool, polls: usize) -> bool {
    regs.request(held);
    for _ in 0..polls {
        if regs.released() != held {
            return true;
        }
    }
    false
}

#[cfg(target_arch = "arm")]
struct Pac;

#[cfg(target_arch = "arm")]
impl ResetRegisters for Pac {
    fn request(&mut self, held: bool) {
        let mut mask = crate::pac::mrcc::MrccGlbRst0::default();
        mask.set_usb0(true);
        if held {
            crate::pac::MRCC0.mrcc_glb_rst0_clr().write(|w| w.set_data(mask.0));
        } else {
            crate::pac::MRCC0.mrcc_glb_rst0_set().write(|w| w.set_data(mask.0));
        }
    }
    fn released(&mut self) -> bool {
        crate::pac::MRCC0.mrcc_glb_rst0().read().usb0()
    }
}

#[cfg(target_arch = "arm")]
pub(crate) fn set_reset_held(held: bool) -> bool {
    request_reset(&mut Pac, held, 1024)
}

#[cfg(test)]
mod tests {
    use super::*;
    struct Register {
        released: bool,
        accepts: bool,
        reads: usize,
        writes: usize,
    }
    impl ResetRegisters for Register {
        fn request(&mut self, held: bool) {
            self.writes += 1;
            if self.accepts {
                self.released = !held;
            }
        }
        fn released(&mut self) -> bool {
            self.reads += 1;
            self.released
        }
    }
    #[test]
    fn rejected_assertion_and_release_are_bounded_after_one_request() {
        for held in [false, true] {
            let mut regs = Register {
                released: held,
                accepts: false,
                reads: 0,
                writes: 0,
            };
            assert!(!request_reset(&mut regs, held, 7));
            assert_eq!((regs.writes, regs.reads), (1, 7));
        }
    }
    #[test]
    fn successful_assertion_and_release_observe_opposite_polarity() {
        for held in [false, true] {
            let mut regs = Register {
                released: held,
                accepts: true,
                reads: 0,
                writes: 0,
            };
            assert!(request_reset(&mut regs, held, 7));
            assert_eq!((regs.writes, regs.reads), (1, 1));
        }
    }
}
