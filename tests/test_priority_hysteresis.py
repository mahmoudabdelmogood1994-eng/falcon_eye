"""
test_priority_hysteresis.py
Verifies the safety-critical property of PriorityHysteresis: it may damp
noise-driven de-escalation, but must NEVER delay an escalation or a
reflex-triggered override. A hysteresis mechanism that silently delayed a
real emergency would be worse than having none at all.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from falcon_eye.multi_target_priority import TargetTrack, PriorityHysteresis


def make_track(tid, slack, risk="ADVISORY"):
    return TargetTrack(target_id=tid, estimated_range=1000.0, relative_angle=0.0,
                        closing_velocity=100.0, ttc=slack, slack=slack, risk_level=risk)


def test_escalation_is_never_delayed():
    h = PriorityHysteresis(hold_time_s=5.0)
    h.update(0.0, [make_track(0, slack=20.0)], reflex_target_id=None)
    # A much more urgent target appears -- must switch immediately, not
    # wait out the 5s hold.
    result = h.update(0.1, [make_track(1, slack=2.0), make_track(0, slack=20.0)], reflex_target_id=None)
    assert result == 1, f"Escalation was delayed! Got {result}, expected immediate switch to target 1"
    print("PASS: escalation is immediate")


def test_reflex_always_overrides_instantly():
    h = PriorityHysteresis(hold_time_s=5.0)
    h.update(0.0, [make_track(0, slack=20.0)], reflex_target_id=None)
    # Reflex fires on a target that isn't even top of the raw sort this
    # cycle -- must still win immediately.
    result = h.update(0.1, [make_track(0, slack=20.0), make_track(2, slack=25.0, risk="CRITICAL_REFLEX")],
                       reflex_target_id=2)
    assert result == 2, f"Reflex was not immediate! Got {result}, expected target 2"
    print("PASS: reflex always overrides instantly")


def test_deescalation_is_damped():
    h = PriorityHysteresis(hold_time_s=1.0)
    h.update(0.0, [make_track(0, slack=5.0)], reflex_target_id=None)
    h.update(0.1, [make_track(1, slack=1.0), make_track(0, slack=5.0)], reflex_target_id=None)
    # Now target 1 clears (becomes less urgent than 0) -- this is a
    # de-escalation and should NOT switch immediately.
    r1 = h.update(0.2, [make_track(0, slack=5.0), make_track(1, slack=6.0)], reflex_target_id=None)
    assert r1 == 1, "De-escalation switched too early (should still be holding target 1)"
    # After the hold time elapses with the proposal persisting, it should switch.
    r2 = h.update(1.3, [make_track(0, slack=5.0), make_track(1, slack=6.0)], reflex_target_id=None)
    assert r2 == 0, f"De-escalation never completed after hold time. Got {r2}"
    print("PASS: de-escalation is damped, then completes after hold time")


if __name__ == "__main__":
    test_escalation_is_never_delayed()
    test_reflex_always_overrides_instantly()
    test_deescalation_is_damped()
    print("\nAll hysteresis safety tests passed.")
