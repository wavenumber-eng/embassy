//! CPU-side bank ownership. Every production transition runs under the same
//! critical section as descriptor publication or completion FIFO consumption.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Phase {
    Idle,
    Reserved(u64),
    Published { id: u64, abandoned: bool },
    Complete { id: u64, length: usize },
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Channel {
    pub enabled: bool,
    pub stalled: bool,
    pub odd: bool,
    pub data1: bool,
    pub generation: u64,
    pub phase: Phase,
    pub completed_bank: bool,
}

impl Channel {
    pub const fn new() -> Self {
        Self {
            enabled: false,
            stalled: false,
            odd: false,
            data1: false,
            generation: 0,
            phase: Phase::Idle,
            completed_bank: false,
        }
    }

    pub fn reserve(&mut self, id: u64) -> Option<u64> {
        if !self.enabled || self.stalled || self.phase != Phase::Idle {
            return None;
        }
        self.phase = Phase::Reserved(id);
        Some(self.generation)
    }

    pub fn publish(&mut self, id: u64, generation: u64) -> bool {
        if self.enabled && !self.stalled && self.generation == generation && self.phase == Phase::Reserved(id) {
            self.phase = Phase::Published { id, abandoned: false };
            true
        } else {
            false
        }
    }

    /// Called only after OWN returned and the exact FIFO bank was captured.
    pub fn complete(&mut self, odd: bool, length: usize) -> bool {
        let Phase::Published { id, abandoned } = self.phase else {
            return false;
        };
        if odd != self.odd {
            return false;
        }
        self.completed_bank = odd;
        self.odd = !odd;
        self.data1 = !self.data1;
        self.phase = if abandoned {
            Phase::Idle
        } else {
            Phase::Complete { id, length }
        };
        true
    }

    pub fn abandon(&mut self, id: u64) {
        match self.phase {
            Phase::Published { id: current, .. } if current == id => {
                self.phase = Phase::Published { id, abandoned: true };
            }
            Phase::Reserved(current) | Phase::Complete { id: current, .. } if current == id => {
                self.phase = Phase::Idle;
            }
            _ => {}
        }
    }

    /// The caller has stopped the engine and accounted for its completion FIFO.
    /// Endpoint changes preserve parity; only a global ODDRST resets it.
    pub fn reclaim_quiesced(&mut self, global_reset: bool) {
        self.generation = self.generation.checked_add(1).expect("USB generation exhausted");
        self.phase = Phase::Idle;
        self.data1 = false;
        self.stalled = false;
        if global_reset {
            self.odd = false;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn enabled() -> Channel {
        let mut channel = Channel::new();
        channel.enabled = true;
        channel
    }

    #[test]
    fn reset_between_reservation_and_publication_rejects_own() {
        let mut c = enabled();
        let generation = c.reserve(7).unwrap();
        c.reclaim_quiesced(true);
        assert!(!c.publish(7, generation));
        assert_eq!(c.phase, Phase::Idle);
    }

    #[test]
    fn disable_between_reservation_and_publication_rejects_own() {
        let mut c = enabled();
        let generation = c.reserve(7).unwrap();
        c.enabled = false;
        assert!(!c.publish(7, generation));
    }

    #[test]
    fn cancellation_does_not_reclaim_a_device_owned_bank() {
        let mut c = enabled();
        let g = c.reserve(1).unwrap();
        assert!(c.publish(1, g));
        c.abandon(1);
        assert!(c.reserve(2).is_none());
        assert!(c.complete(false, 17));
        assert_eq!(c.phase, Phase::Idle);
        assert!(c.odd);
        assert!(c.reserve(2).is_some());
    }

    #[test]
    fn full_completion_stays_reserved_until_consumed_or_abandoned() {
        let mut c = enabled();
        let g = c.reserve(1).unwrap();
        assert!(c.publish(1, g));
        assert!(c.complete(false, 64));
        assert_eq!(c.phase, Phase::Complete { id: 1, length: 64 });
        assert!(c.reserve(2).is_none());
        c.abandon(1);
        assert!(c.reserve(2).is_some());
    }

    #[test]
    fn unrelated_or_wrong_bank_completions_cannot_free_a_transfer() {
        let mut c = enabled();
        assert!(!c.complete(false, 7));
        let g = c.reserve(8).unwrap();
        assert!(c.publish(8, g));
        assert!(!c.complete(true, 7));
        assert_eq!(
            c.phase,
            Phase::Published {
                id: 8,
                abandoned: false
            }
        );
    }

    #[test]
    fn per_endpoint_disable_preserves_both_possible_hardware_bank_positions() {
        for odd in [false, true] {
            let mut c = enabled();
            c.odd = odd;
            c.reclaim_quiesced(false);
            assert_eq!(c.odd, odd);
            assert!(!c.data1);
        }
    }

    #[test]
    fn old_future_drop_cannot_cancel_replacement_after_quiescent_reuse() {
        let mut c = enabled();
        let g = c.reserve(1).unwrap();
        assert!(c.publish(1, g));
        c.reclaim_quiesced(false);
        let g = c.reserve(2).unwrap();
        assert!(c.publish(2, g));
        c.abandon(1);
        assert_eq!(
            c.phase,
            Phase::Published {
                id: 2,
                abandoned: false
            }
        );
    }
}
