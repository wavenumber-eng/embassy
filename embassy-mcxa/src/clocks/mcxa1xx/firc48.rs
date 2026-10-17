//! Nominal fixed48MHz peripheral output; USB accuracy qualification is separate.

use crate::pac::scg::{Fircacc, Firccsr, FirccsrLk, Fircerr, Fircvld, FreqSel};

/// Startup is bounded in register polls, not in calibrated time.
pub(super) const STARTUP_POLLS: usize = 100_000;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub(super) enum Error {
    Fault,
    SelectedButDisabled,
    Configuration,
    Timeout,
}

/// The clock initializer serializes CPU users before entering this protocol.
pub(super) trait Registers {
    fn control(&mut self) -> Firccsr;
    fn write_control(&mut self, value: Firccsr);
    fn frequency(&mut self) -> FreqSel;
    fn select_48mhz(&mut self);
}

pub(super) struct PacRegisters;

impl Registers for PacRegisters {
    fn control(&mut self) -> Firccsr {
        crate::pac::SCG0.firccsr().read()
    }

    fn write_control(&mut self, value: Firccsr) {
        crate::pac::SCG0.firccsr().write_value(value);
    }

    fn frequency(&mut self) -> FreqSel {
        crate::pac::SCG0.firccfg().read().freq_sel()
    }

    fn select_48mhz(&mut self) {
        crate::pac::SCG0
            .firccfg()
            .modify(|w| w.set_freq_sel(FreqSel::Firc45_48mhz));
    }
}

fn update_control(regs: &mut impl Registers, update: impl FnOnce(&mut Firccsr)) {
    let mut value = regs.control();
    update(&mut value);
    // FIRCERR is W1C even though the PAC exposes an ordinary RW accessor.
    // Never acknowledge an error as a side effect of clock configuration.
    value.set_fircerr(Fircerr::ErrorNotDetected);
    regs.write_control(value);
}

pub(super) fn enable(regs: &mut impl Registers, polls: usize) -> Result<(), Error> {
    let initial = regs.control();
    if initial.fircerr() != Fircerr::ErrorNotDetected {
        return Err(Error::Fault);
    }
    if initial.fircsel() && !initial.fircen() {
        return Err(Error::SelectedButDisabled);
    }

    let restore_lock = if initial.fircen() {
        initial.lk()
    } else {
        FirccsrLk::WriteDisabled
    };
    update_control(regs, |w| w.set_lk(FirccsrLk::WriteEnabled));
    let configured = (|| {
        let unlocked = regs.control();
        if unlocked.fircerr() != Fircerr::ErrorNotDetected {
            return Err(Error::Fault);
        }
        if unlocked.lk() != FirccsrLk::WriteEnabled || unlocked.fircen() != initial.fircen() {
            return Err(Error::Configuration);
        }
        // The fixed48MHz output is independent of the already running FIRC's
        // FREQ_SEL. Reclocking a running FIRC could change the CPU clock.
        if !initial.fircen() {
            regs.select_48mhz();
            if regs.frequency() != FreqSel::Firc45_48mhz {
                return Err(Error::Configuration);
            }
        }
        update_control(regs, |w| {
            w.set_fircen(true);
            w.set_firc_sclk_periph_en(true);
        });
        Ok(())
    })();
    // Restore an existing owner's lock state; freeze a newly started source.
    // Leave its enable state intact on failure: it may feed the system clock.
    update_control(regs, |w| w.set_lk(restore_lock));
    configured?;

    for _ in 0..polls {
        let value = regs.control();
        if value.fircerr() != Fircerr::ErrorNotDetected {
            return Err(Error::Fault);
        }
        if value.fircen()
            && value.firc_sclk_periph_en()
            && value.lk() == restore_lock
            && value.fircvld() == Fircvld::EnabledAndValid
            && value.fircacc() == Fircacc::EnabledAndValid
        {
            return Ok(());
        }
    }
    Err(Error::Timeout)
}
