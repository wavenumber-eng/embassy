//! Native contract tests for the actual A156 FIRC48 initialization protocol.
pub use nxp_pac as pac;

#[allow(dead_code)]
#[path = "../../../src/clocks/mcxa1xx/firc48.rs"]
mod firc48;

#[cfg(test)]
mod tests {
    use super::firc48::{Error, Registers, enable};
    use super::pac::scg::{Fircacc, Firccsr, FirccsrLk, Fircerr, FircerrIe, Fircsten, Fircvld, FreqSel};

    struct Oscillator {
        control: Firccsr,
        frequency: FreqSel,
        reads: usize,
        writes: Vec<Firccsr>,
        frequency_writes: usize,
        fault_on_write: Option<usize>,
        ready_after: Option<usize>,
        ignore_frequency: bool,
        deny_unlock: bool,
    }

    impl Oscillator {
        fn running() -> Self {
            let mut control = Firccsr::default();
            control.set_fircen(true);
            control.set_lk(FirccsrLk::WriteDisabled);
            control.set_fircvld(Fircvld::EnabledAndValid);
            control.set_fircacc(Fircacc::EnabledAndValid);
            Self {
                control,
                frequency: FreqSel::Firc90_96mhz,
                reads: 0,
                writes: Vec::new(),
                frequency_writes: 0,
                fault_on_write: None,
                ready_after: None,
                ignore_frequency: false,
                deny_unlock: false,
            }
        }
    }

    impl Registers for Oscillator {
        fn control(&mut self) -> Firccsr {
            self.reads += 1;
            if self.ready_after.is_some_and(|n| self.reads >= n) {
                self.control.set_fircvld(Fircvld::EnabledAndValid);
                self.control.set_fircacc(Fircacc::EnabledAndValid);
            }
            self.control
        }

        fn write_control(&mut self, value: Firccsr) {
            self.writes.push(value);
            if self.fault_on_write == Some(self.writes.len()) {
                self.control.set_fircerr(Fircerr::ErrorDetected);
            }
            let previous = self.control;
            if self.deny_unlock && value.lk() == FirccsrLk::WriteEnabled {
                return;
            }
            self.control = value;
            // Hardware status is not changed by writing back its sampled value.
            self.control.set_fircvld(previous.fircvld());
            self.control.set_fircacc(previous.fircacc());
            self.control.set_fircsel(previous.fircsel());
            // W1C: zero preserves a fault, including one arriving after the read.
            self.control.set_fircerr(if value.fircerr() == Fircerr::ErrorDetected {
                Fircerr::ErrorNotDetected
            } else {
                previous.fircerr()
            });
        }

        fn frequency(&mut self) -> FreqSel {
            self.frequency
        }

        fn select_48mhz(&mut self) {
            assert!(!self.control.fircen(), "a live system source must not be retuned");
            self.frequency_writes += 1;
            if !self.ignore_frequency {
                self.frequency = FreqSel::Firc45_48mhz;
            }
        }
    }

    #[test]
    fn running_source_keeps_frequency_trim_interrupts_and_stop_policy() {
        let mut regs = Oscillator::running();
        regs.control.set_fircsel(true);
        regs.control.set_firctren(true);
        regs.control.set_firctrup(true);
        regs.control.set_fircsten(Fircsten::EnabledInStopModes);
        regs.control.set_fircerr_ie(FircerrIe::ErrorDetected);
        let mut expected = regs.control;
        expected.set_firc_sclk_periph_en(true);
        assert_eq!(enable(&mut regs, 4), Ok(()));
        assert_eq!(regs.frequency, FreqSel::Firc90_96mhz);
        assert_eq!(regs.frequency_writes, 0);
        assert_eq!(regs.control.0, expected.0);
    }

    #[test]
    fn existing_unlocked_source_stays_unlocked() {
        let mut regs = Oscillator::running();
        regs.control.set_lk(FirccsrLk::WriteEnabled);
        assert_eq!(enable(&mut regs, 4), Ok(()));
        assert_eq!(regs.control.lk(), FirccsrLk::WriteEnabled);
    }

    #[test]
    fn disabled_source_starts_at_48mhz_without_enabling_unrelated_output() {
        let mut regs = Oscillator::running();
        regs.control.set_fircen(false);
        assert_eq!(enable(&mut regs, 4), Ok(()));
        assert_eq!(regs.frequency, FreqSel::Firc45_48mhz);
        assert_eq!(regs.frequency_writes, 1);
        assert!(!regs.control.firc_fclk_periph_en());
        assert_eq!(regs.control.lk(), FirccsrLk::WriteDisabled);
    }

    #[test]
    fn inconsistent_selected_source_is_rejected_without_writes() {
        let mut regs = Oscillator::running();
        regs.control.set_fircen(false);
        regs.control.set_fircsel(true);
        assert_eq!(enable(&mut regs, 4), Err(Error::SelectedButDisabled));
        assert!(regs.writes.is_empty());
        assert_eq!(regs.frequency_writes, 0);
    }

    #[test]
    fn preexisting_fault_is_preserved_and_rejected_without_writes() {
        let mut regs = Oscillator::running();
        regs.control.set_fircerr(Fircerr::ErrorDetected);
        assert_eq!(enable(&mut regs, 4), Err(Error::Fault));
        assert!(regs.writes.is_empty());
        assert_eq!(regs.control.fircerr(), Fircerr::ErrorDetected);
    }

    #[test]
    fn fault_arriving_between_each_read_and_write_is_never_cleared() {
        for write in 1..=3 {
            let mut regs = Oscillator::running();
            regs.fault_on_write = Some(write);
            assert_eq!(enable(&mut regs, 4), Err(Error::Fault));
            assert_eq!(regs.control.fircerr(), Fircerr::ErrorDetected);
            assert!(regs.writes.iter().all(|w| w.fircerr() == Fircerr::ErrorNotDetected));
            assert_eq!(regs.control.lk(), FirccsrLk::WriteDisabled);
        }
    }

    #[test]
    fn validity_and_accuracy_are_both_required_and_wait_is_bounded() {
        for valid in [false, true] {
            let mut regs = Oscillator::running();
            regs.control.set_fircvld(if valid {
                Fircvld::EnabledAndValid
            } else {
                Fircvld::NotEnabledOrNotValid
            });
            regs.control.set_fircacc(Fircacc::NotEnabledOrNotValid);
            assert_eq!(enable(&mut regs, 7), Err(Error::Timeout));
            assert_eq!(regs.reads, 12);
            assert!(regs.control.fircen(), "failure must not stop a system source");
        }
    }

    #[test]
    fn delayed_readiness_is_observed_without_reclocking() {
        let mut regs = Oscillator::running();
        regs.control.set_fircacc(Fircacc::NotEnabledOrNotValid);
        regs.ready_after = Some(9);
        assert_eq!(enable(&mut regs, 7), Ok(()));
        assert_eq!(regs.frequency_writes, 0);
    }

    #[test]
    fn rejected_frequency_write_does_not_publish_or_enable_the_source() {
        let mut regs = Oscillator::running();
        regs.control.set_fircen(false);
        regs.ignore_frequency = true;
        assert_eq!(enable(&mut regs, 4), Err(Error::Configuration));
        assert!(!regs.control.fircen());
        assert_eq!(regs.control.lk(), FirccsrLk::WriteDisabled);
    }

    #[test]
    fn rejected_unlock_prevents_frequency_changes() {
        let mut regs = Oscillator::running();
        regs.control.set_fircen(false);
        regs.deny_unlock = true;
        assert_eq!(enable(&mut regs, 4), Err(Error::Configuration));
        assert_eq!(regs.frequency_writes, 0);
    }
}
