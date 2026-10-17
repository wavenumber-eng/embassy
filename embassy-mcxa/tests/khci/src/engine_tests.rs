use std::collections::VecDeque;

use crate::controller::*;
use crate::io::{Io, Register as R};
use crate::storage::{OWN, Storage, slot};
use crate::transfer::Phase;

struct Fake {
    ctl: u8,
    inten: u8,
    flags: u8,
    control: u8,
    address: u8,
    endpoints: [u8; 16],
    fifo: VecDeque<u8>,
    ignore_stop: bool,
    reset_after_ack: bool,
    barriers: usize,
    acknowledgments: Vec<u8>,
    token_on_ctl_write: Option<u8>,
    reset_on_enable: bool,
}

impl Fake {
    fn new() -> Self {
        Self {
            ctl: 1,
            inten: 255,
            flags: 0,
            control: 16,
            address: 37,
            endpoints: [0; 16],
            fifo: VecDeque::new(),
            ignore_stop: false,
            reset_after_ack: false,
            barriers: 0,
            acknowledgments: Vec::new(),
            token_on_ctl_write: None,
            reset_on_enable: false,
        }
    }
}

impl Io for Fake {
    fn read(&mut self, r: R) -> u8 {
        match r {
            R::Ctl => self.ctl,
            R::Control => self.control,
            R::Addr => self.address,
            R::Istat => self.flags | if self.fifo.is_empty() { 0 } else { TOKEN },
            R::Inten => self.inten,
            R::Stat => *self.fifo.front().unwrap(),
            R::Endpoint(n) => self.endpoints[n],
            _ => 0,
        }
    }
    fn write(&mut self, r: R, value: u8) {
        if r == R::Ctl
            && let Some(stat) = self.token_on_ctl_write.take()
        {
            self.fifo.push_back(stat);
        }
        if r == R::Ctl && value & 1 != 0 && self.ctl & 1 == 0 && self.reset_on_enable {
            self.flags |= RESET;
        }
        match r {
            R::Ctl if !self.ignore_stop || value & 1 != 0 => self.ctl = value,
            R::Control => self.control = value,
            R::Addr => self.address = value,
            R::Inten => self.inten = value,
            R::Endpoint(n) => self.endpoints[n] = value,
            R::Istat => {
                self.acknowledgments.push(value);
                self.flags &= !value;
                if value & TOKEN != 0 {
                    self.fifo.pop_front();
                    if self.reset_after_ack {
                        self.flags |= RESET;
                        self.reset_after_ack = false;
                    }
                }
            }
            _ => {}
        }
    }
    fn barrier(&mut self) {
        self.barriers += 1;
    }
}

fn complete(e: &mut Engine<'_, Fake>, index: usize, odd: bool, length: usize, pid: u8) {
    e.storage
        .set_control(slot(index, odd), ((length as u32) << 16) | ((pid as u32) << 2));
    e.io.fifo
        .push_back(((index / 2) as u8) << 4 | if index % 2 == 1 { 8 } else { 0 } | if odd { 4 } else { 0 });
}

#[test]
fn endpoint_disable_drains_unrelated_completion_and_preserves_address_pullup_and_parity() {
    for odd in [false, true] {
        let storage = Storage::new();
        let mut state = Inner::new();
        state.active = true;
        let mut e = Engine {
            io: Fake::new(),
            state: &mut state,
            storage: &storage,
        };
        e.state.channels[2].enabled = true;
        e.state.channels[2].odd = odd;
        e.state.channels[5].enabled = true;
        let (id, _) = e.begin(5, 3, Some(&[1, 2, 3]), None).unwrap();
        complete(&mut e, 5, false, 3, 9);
        e.endpoint(2, false, false).unwrap();
        assert_eq!(e.state.channels[5].phase, Phase::Complete { id, length: 3 });
        assert_eq!(e.state.channels[2].odd, odd);
        assert_eq!(e.io.address, 37);
        assert_eq!(e.io.control, 16);
        assert_eq!(e.io.ctl & 1, 1);
        assert!(e.io.fifo.is_empty());
        e.endpoint(2, true, false).unwrap();
        e.begin(2, 64, None, None).unwrap();
        assert_ne!(storage.control(slot(2, odd)) & OWN, 0);
    }
}

#[test]
fn cancel_then_completion_does_not_publish_a_stale_result_to_replacement() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[3].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    let (old, _) = e.begin(3, 1, Some(&[4]), None).unwrap();
    e.state.channels[3].abandon(old);
    assert!(e.begin(3, 1, Some(&[8]), None).is_none());
    complete(&mut e, 3, false, 1, 9);
    e.interrupt().unwrap();
    let (new, _) = e.begin(3, 1, Some(&[8]), None).unwrap();
    assert_ne!(old, new);
    assert!(e.state.channels[3].odd);
    e.state.channels[3].abandon(old);
    assert_eq!(
        e.state.channels[3].phase,
        Phase::Published {
            id: new,
            abandoned: false
        }
    );
}

#[test]
fn reset_arriving_while_draining_does_not_restore_an_endpoint_enable_or_old_address() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[5].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.begin(5, 0, Some(&[]), None).unwrap();
    complete(&mut e, 5, false, 0, 9);
    e.io.reset_after_ack = true;
    e.endpoint(2, true, false).unwrap();
    assert!(!e.state.channels[2].enabled);
    assert_eq!(e.io.address, 0);
    assert_ne!(e.state.events & RESET_EVENT, 0);
    assert_eq!(e.io.control, 16);
    assert_ne!(storage.control(slot(0, false)) & OWN, 0);
}

#[test]
fn setup_preempts_in_and_copies_bytes_before_rearming_out() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    state.channels[1].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    e.begin(1, 7, Some(&[1; 7]), Some(0)).unwrap();
    let setup = [0, 5, 42, 0, 0, 0, 0, 0];
    storage.copy_in(slot(0, false), &setup);
    complete(&mut e, 0, false, 8, 13);
    e.interrupt().unwrap();
    assert!(e.state.setup_pending);
    assert_eq!(e.state.setup, setup);
    assert_eq!(e.state.setup_generation, 1);
    assert_eq!(e.state.channels[1].phase, Phase::Idle);
    assert_eq!(storage.control(slot(1, false)), 0);
    assert_ne!(storage.control(slot(0, true)) & OWN, 0);
    e.set_address(0, 99).unwrap();
    assert_eq!(e.io.address, 37);
}

#[test]
fn setup_and_old_in_completion_in_same_fifo_invalidate_the_old_completion() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    state.channels[1].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    e.begin(1, 0, Some(&[]), Some(0)).unwrap();
    complete(&mut e, 1, false, 0, 9);
    storage.copy_in(slot(0, false), &[0; 8]);
    complete(&mut e, 0, false, 8, 13);
    e.interrupt().unwrap();
    assert_eq!(e.state.channels[1].phase, Phase::Idle);
    assert!(e.state.setup_pending);
}

#[test]
fn stop_failure_quarantines_descriptors_and_disables_future_publication() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[3].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.begin(3, 4, Some(&[7; 4]), None).unwrap();
    let before = storage.control(slot(3, false));
    e.io.ignore_stop = true;
    assert_eq!(e.endpoint(3, false, false), Err(Fault::StopTimeout));
    assert_eq!(storage.control(slot(3, false)), before);
    assert_eq!(e.io.control & 16, 0);
    assert_eq!(e.io.inten, 0);
    assert!(e.begin(3, 1, Some(&[9]), None).is_none());
}

#[test]
fn dma_length_overflow_fails_before_copy_and_never_reuses_the_bank() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[2].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.begin(2, 64, None, None).unwrap();
    complete(&mut e, 2, false, 65, 1);
    assert_eq!(e.interrupt(), Err(Fault::Length));
    assert!(!e.state.active);
    assert!(!e.io.fifo.is_empty());
}

#[test]
fn own_not_returned_is_a_fault_not_a_completion() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[3].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.begin(3, 1, Some(&[9]), None).unwrap();
    e.io.fifo.push_back(0x18);
    assert_eq!(e.interrupt(), Err(Fault::Ownership));
    assert!(matches!(e.state.channels[3].phase, Phase::Published { .. }));
    let before = storage.control(slot(3, false));
    assert_eq!(e.set_address(0, 42), Err(Fault::Ownership));
    assert_eq!(e.io.ctl & 1, 0);
    assert_eq!(storage.control(slot(3, false)), before);
    assert_eq!(e.io.address, 37);
}

#[test]
fn early_out_status_unblocks_new_setup_without_acknowledging_the_old_in() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    state.channels[1].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    let (id, _) = e.begin(1, 64, Some(&[7; 64]), Some(0)).unwrap();
    complete(&mut e, 0, false, 0, 1);
    e.interrupt().unwrap();
    let mut done = false;
    assert_eq!(
        e.control_in_poll(id, 0, true, &mut done),
        Err(embassy_usb_driver::EndpointError::Disabled)
    );
    assert!(!done);
    assert_ne!(storage.control(slot(0, true)) & OWN, 0);
    storage.copy_in(slot(0, true), &[0; 8]);
    complete(&mut e, 0, true, 8, 13);
    e.interrupt().unwrap();
    assert!(e.setup_poll().is_some());
    assert_eq!(e.state.channels[1].phase, Phase::Idle);
}

#[test]
fn setup_wait_rearms_out_after_cancelled_data_out() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    complete(&mut e, 0, false, 7, 1);
    e.interrupt().unwrap();
    assert!(e.state.out_packet.is_some());
    assert!(e.setup_poll().is_none());
    assert!(e.state.out_packet.is_none());
    assert_ne!(storage.control(slot(0, true)) & OWN, 0);
}

#[test]
fn short_control_out_reports_actual_length_and_rearms_setup_storage() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    storage.copy_in(slot(0, false), &[5, 6, 7]);
    complete(&mut e, 0, false, 3, 1);
    e.interrupt().unwrap();
    let mut data = [0; 7];
    assert_eq!(e.control_out_poll(0, &mut data), Ok(Some(3)));
    assert_eq!(&data[..3], &[5, 6, 7]);
    assert_ne!(storage.control(slot(0, true)) & OWN, 0);
}

#[test]
fn setup_arriving_at_publication_boundary_prevents_old_in_own() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    state.channels[1].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    storage.copy_in(slot(0, false), &[0; 8]);
    storage.set_control(slot(0, false), (8 << 16) | (13 << 2));
    e.io.token_on_ctl_write = Some(0);
    let notifications = e.state.notifications;
    assert!(e.begin(1, 7, Some(&[9; 7]), Some(0)).is_none());
    assert_eq!(storage.control(slot(1, false)) & OWN, 0);
    assert_eq!(storage.control(slot(1, true)) & OWN, 0);
    assert_eq!(e.state.setup_generation, 1);
    assert!(
        e.state.notifications > notifications,
        "task-side drain must notify other waiters"
    );
}

#[test]
fn queued_new_setup_prevents_an_old_request_reject_from_stalling_it() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    state.channels[1].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    storage.copy_in(slot(0, false), &[0; 8]);
    complete(&mut e, 0, false, 8, 13);
    e.reject(0).unwrap();
    assert_eq!(e.io.endpoints[0] & 2, 0);
    assert!(e.state.setup_pending);
    assert_ne!(storage.control(slot(0, true)) & OWN, 0);
}

#[test]
fn rejection_keeps_setup_bank_owned_and_new_setup_clears_stall() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.channels[0].enabled = true;
    state.channels[1].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.prime_out();
    e.reject(0).unwrap();
    assert_ne!(e.io.endpoints[0] & 2, 0);
    assert_ne!(storage.control(slot(0, false)) & OWN, 0);
    storage.copy_in(slot(0, false), &[0; 8]);
    complete(&mut e, 0, false, 8, 13);
    e.interrupt().unwrap();
    assert_eq!(e.io.endpoints[0] & 2, 0);
    assert!(e.state.setup_pending);
    assert_ne!(storage.control(slot(0, true)) & OWN, 0);
}

#[test]
fn suspend_then_resume_before_poll_cannot_leave_the_stack_suspended() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.events = 0;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.io.flags = SLEEP;
    e.interrupt().unwrap();
    e.io.flags = RESUME;
    e.interrupt().unwrap();
    assert_eq!(e.state.next_event(), None);
    e.io.flags = SLEEP;
    e.interrupt().unwrap();
    assert_eq!(e.state.next_event(), Some(embassy_usb_driver::Event::Suspend));
    e.io.flags = RESUME;
    e.interrupt().unwrap();
    assert_eq!(e.state.next_event(), Some(embassy_usb_driver::Event::Resume));
}

#[test]
fn reset_invalidates_old_sleep_and_resume_events_in_the_same_snapshot() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.events = 0;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.io.flags = SLEEP;
    e.interrupt().unwrap();
    assert_eq!(e.state.next_event(), Some(embassy_usb_driver::Event::Suspend));
    e.io.flags = RESET | SLEEP | RESUME;
    e.interrupt().unwrap();
    assert_eq!(e.state.next_event(), Some(embassy_usb_driver::Event::Reset));
    assert_eq!(e.state.next_event(), None);
    assert_eq!(e.io.flags & (SLEEP | RESUME), 0);
}

#[test]
fn recognized_reset_is_acknowledged_after_enable_without_reentering_reset_service() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.events = 0;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.io.flags = RESET;
    e.io.reset_on_enable = true;
    e.interrupt().unwrap();
    assert_eq!(e.io.flags & RESET, 0);
    assert!(!e.state.reset_ack_pending);
    let resets = e.state.resets;
    e.interrupt().unwrap();
    assert_eq!(e.state.resets, resets);
}

#[test]
fn ordinary_publication_does_not_hide_a_new_reset_generated_after_resume() {
    let storage = Storage::new();
    let mut state = Inner::new();
    state.active = true;
    state.events = 0;
    state.channels[3].enabled = true;
    let mut e = Engine {
        io: Fake::new(),
        state: &mut state,
        storage: &storage,
    };
    e.io.reset_on_enable = true;
    assert!(e.begin(3, 0, Some(&[]), None).is_some());
    assert_ne!(e.io.flags & RESET, 0, "unobserved resets must remain visible");
    assert_eq!(e.state.events & RESET_EVENT, 0);
}
