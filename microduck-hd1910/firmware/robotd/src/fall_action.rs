//! What happens once the robot is down while a policy drives it: `[safety] fall_action`.
//!
//! Upstream leaves a fall to the policy. The velstand network is trained to fall protectively
//! and get back up, and the safety layer deliberately gates nothing on the fall verdict
//! (robotd-design §2.4). That holds on the robot the policy was trained for. Where the learned
//! get-up does not land, the policy tries, fails and tries again, legs thrashing against the
//! floor — the failure the limp-fall block in `main.rs` describes for a dynamic fall.
//!
//! So the operator chooses, per robot:
//!
//! - `policy` (default): as upstream, nothing happens;
//! - `relax`: torque off as soon as the fall verdict has latched; the robot stays limp until
//!   someone stands it up and enables it again;
//! - `relax_after_retry`: the policy gets `fall_retry_ms` to get the robot up first. The clock
//!   only resets once the robot has read upright for `fall_upright_ms`, so a policy that nearly
//!   makes it and drops back cannot keep trying forever.
//!
//! This is the decision only. The loop turns `true` into an ordinary `robot.relax`, and owns
//! what counts as eligible: a policy driving, with no skill, sit, shutdown or limp-fall
//! sequence in charge of the robot.

use crate::params::FallAction;
use std::time::{Duration, Instant};

pub struct FallGuard {
    action: FallAction,
    retry: Duration,
    upright: Duration,
    /// First tick of the current fall, while eligible.
    down_since: Option<Instant>,
    /// First tick of the current upright stretch inside a fall.
    up_since: Option<Instant>,
}

impl FallGuard {
    pub fn new(action: FallAction, retry: Duration, upright: Duration) -> Self {
        Self {
            action,
            retry,
            upright,
            down_since: None,
            up_since: None,
        }
    }

    /// One control tick. `fallen` is the debounced fall verdict; `eligible` says a policy is
    /// driving with nothing deliberate in charge. True means: relax now.
    pub fn observe(&mut self, now: Instant, fallen: bool, eligible: bool) -> bool {
        if self.action == FallAction::Policy || !eligible {
            self.down_since = None;
            self.up_since = None;
            return false;
        }
        if fallen {
            self.up_since = None;
            let since = *self.down_since.get_or_insert(now);
            let due = match self.action {
                FallAction::Relax => true,
                FallAction::RelaxAfterRetry => now.saturating_duration_since(since) >= self.retry,
                FallAction::Policy => false,
            };
            if due {
                self.down_since = None;
            }
            return due;
        }
        if self.down_since.is_some() {
            let up = *self.up_since.get_or_insert(now);
            if now.saturating_duration_since(up) >= self.upright {
                self.down_since = None;
                self.up_since = None;
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RETRY: Duration = Duration::from_millis(4000);
    const UPRIGHT: Duration = Duration::from_millis(1000);

    fn at(t0: Instant, ms: u64) -> Instant {
        t0 + Duration::from_millis(ms)
    }

    #[test]
    fn policy_leaves_every_fall_to_the_policy() {
        let mut guard = FallGuard::new(FallAction::Policy, RETRY, UPRIGHT);
        let t0 = Instant::now();
        for step in 0..=100 {
            assert!(!guard.observe(at(t0, step * 100), true, true));
        }
    }

    #[test]
    fn relax_fires_on_the_first_eligible_fallen_tick() {
        let mut guard = FallGuard::new(FallAction::Relax, RETRY, UPRIGHT);
        let t0 = Instant::now();
        assert!(!guard.observe(t0, false, true), "upright and driving");
        assert!(
            !guard.observe(at(t0, 20), true, false),
            "down, but a skill or sit has it"
        );
        assert!(guard.observe(at(t0, 40), true, true));
        // The relax takes the robot out of driving: nothing more until it drives again.
        assert!(!guard.observe(at(t0, 60), true, false));
    }

    #[test]
    fn retry_gives_the_policy_its_window_then_fires() {
        let mut guard = FallGuard::new(FallAction::RelaxAfterRetry, RETRY, UPRIGHT);
        let t0 = Instant::now();
        assert!(!guard.observe(t0, true, true));
        assert!(!guard.observe(at(t0, 3980), true, true));
        assert!(guard.observe(at(t0, 4000), true, true));
    }

    #[test]
    fn retry_ignores_brief_upright_readings_inside_a_fall() {
        let mut guard = FallGuard::new(FallAction::RelaxAfterRetry, RETRY, UPRIGHT);
        let t0 = Instant::now();
        assert!(!guard.observe(t0, true, true));
        // Rocking up for less than `fall_upright_ms` does not reset the clock.
        assert!(!guard.observe(at(t0, 1000), false, true));
        assert!(!guard.observe(at(t0, 1900), false, true));
        assert!(!guard.observe(at(t0, 2000), true, true));
        assert!(
            guard.observe(at(t0, 4000), true, true),
            "counted from the first fallen tick"
        );
    }

    #[test]
    fn retry_resets_once_the_robot_has_stood_long_enough() {
        let mut guard = FallGuard::new(FallAction::RelaxAfterRetry, RETRY, UPRIGHT);
        let t0 = Instant::now();
        assert!(!guard.observe(t0, true, true));
        assert!(!guard.observe(at(t0, 1000), false, true));
        // Upright for a full `fall_upright_ms`: it got up, and the next fall starts afresh.
        assert!(!guard.observe(at(t0, 2000), false, true));
        assert!(!guard.observe(at(t0, 2500), true, true));
        assert!(!guard.observe(at(t0, 6480), true, true));
        assert!(guard.observe(at(t0, 6500), true, true));
    }

    #[test]
    fn an_ineligible_stretch_restarts_the_window() {
        let mut guard = FallGuard::new(FallAction::RelaxAfterRetry, RETRY, UPRIGHT);
        let t0 = Instant::now();
        assert!(!guard.observe(t0, true, true));
        // An LB get-up skill takes over for a while and hands back with the robot still down.
        assert!(!guard.observe(at(t0, 3000), true, false));
        assert!(!guard.observe(at(t0, 6000), true, false));
        assert!(!guard.observe(at(t0, 6020), true, true));
        assert!(!guard.observe(at(t0, 10000), true, true));
        assert!(guard.observe(at(t0, 10020), true, true));
    }
}
