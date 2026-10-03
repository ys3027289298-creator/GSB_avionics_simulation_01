"""Regression tests for the idempotent ignition state machine.

All event streams are constructed locally — no RocketPy, no serial ports, no
sockets, no external services. Pseudo-random streams use fixed seeds, so every
run is bit-for-bit reproducible:

    python3 -m unittest test_ignition_fsm -v
"""

import random
import unittest

from ignition_fsm import (
    Action,
    IgnitionFSM,
    State,
    EVENT_APOGEE,
    EVENT_PYRO1,
    EVENT_PYRO2,
)

MAX_ALT_M = 450.0
SEED = 20261003


def make_fsm(**overrides):
    params = dict(
        max_alt_m=MAX_ALT_M,
        dedup_window_s=0.5,
        dedup_alt_tolerance_m=5.0,
        min_pyro_separation_s=2.0,
        pyro2_enabled=True,
    )
    params.update(overrides)
    return IgnitionFSM(**params)


def nominal_events():
    return [
        (EVENT_APOGEE, 290.0, 18.50),
        (EVENT_PYRO1, 289.5, 18.55),
        (EVENT_PYRO2, 98.0, 24.10),
    ]


def pseudo_random_stream(seed=SEED):
    """One deterministic stream mixing valid events with every anomaly class."""
    rng = random.Random(seed)
    events = []

    apogee_t, apogee_alt = 18.5, 290.0

    # Sensor jitter: a cluster of slightly different APOGEE reports.
    for i in range(6):
        events.append(
            (
                EVENT_APOGEE,
                apogee_alt + rng.gauss(0.0, 0.5),
                apogee_t + 0.05 * i + rng.uniform(0.0, 0.02),
            )
        )

    # Identical timestamp + value: exact retransmission of the first report.
    events.append(events[0])

    # PYRO1: fire, exact retransmission, retry inside the dedup window,
    # then a genuine double fire with conflicting parameters.
    pyro1_t = apogee_t + 0.4
    events.append((EVENT_PYRO1, apogee_alt - 0.5, pyro1_t))
    events.append((EVENT_PYRO1, apogee_alt - 0.5, pyro1_t))
    events.append((EVENT_PYRO1, apogee_alt - 0.6, pyro1_t + 0.2))
    events.append((EVENT_PYRO1, 150.0, pyro1_t + 5.0))

    # Out-of-bounds altitude, stale timestamp, unknown event type.
    events.append((EVENT_PYRO2, -3.0, pyro1_t + 6.0))
    events.append((EVENT_PYRO2, 98.0, pyro1_t + 4.0))
    events.append(("BOOM", 100.0, pyro1_t + 6.0))

    # Valid PYRO2 with sufficient separation closes the sequence.
    events.append((EVENT_PYRO2, 98.0, apogee_t + 6.0))
    return events


def run_stream(events):
    fsm = make_fsm()
    decisions = [fsm.handle_event(kind, alt, t) for kind, alt, t in events]
    return fsm, decisions


class TestNominalSequence(unittest.TestCase):
    def test_nominal_dual_deploy_accepted(self):
        fsm, decisions = run_stream(nominal_events())
        self.assertEqual([d.action for d in decisions], [Action.ACCEPT] * 3)
        self.assertEqual(fsm.state, State.PYRO2_FIRED)
        self.assertFalse(fsm.degraded)
        self.assertEqual(fsm.anomalies, [])
        self.assertEqual(fsm.apogee, (18.50, 290.0))
        self.assertEqual(fsm.pyro1, (18.55, 289.5))
        self.assertEqual(fsm.pyro2, (24.10, 98.0))


class TestIdempotency(unittest.TestCase):
    def test_identical_timestamps_are_idempotent(self):
        fsm = make_fsm()
        first = fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        replay = fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        self.assertEqual(first.action, Action.ACCEPT)
        self.assertEqual(replay.action, Action.DUPLICATE)
        self.assertEqual(replay.kind, "exact_retransmission")
        # A different event on the same tick is processed in arrival order.
        pyro = fsm.handle_event(EVENT_PYRO1, 289.5, 18.5)
        self.assertEqual(pyro.action, Action.ACCEPT)
        self.assertEqual(fsm.state, State.PYRO1_FIRED)

    def test_sensor_jitter_cluster_deduplicated(self):
        rng = random.Random(SEED)
        fsm = make_fsm()
        actions = []
        for i in range(8):
            alt = 290.0 + rng.gauss(0.0, 0.4)
            t = 18.5 + 0.05 * i + rng.uniform(0.0, 0.02)
            actions.append(fsm.handle_event(EVENT_APOGEE, alt, t).action)
        self.assertEqual(actions[0], Action.ACCEPT)
        self.assertEqual(actions[1:], [Action.DUPLICATE] * 7)
        self.assertFalse(fsm.degraded)
        self.assertEqual(fsm.anomalies, [])

    def test_retry_after_rejected_frame_still_works(self):
        fsm = make_fsm()
        fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        rejected = fsm.handle_event(EVENT_PYRO1, -1.0, 18.6)
        self.assertEqual(rejected.action, Action.REJECT)
        retried = fsm.handle_event(EVENT_PYRO1, 289.5, 18.6)
        self.assertEqual(retried.action, Action.ACCEPT)
        self.assertEqual(fsm.state, State.PYRO1_FIRED)


class TestDoubleFireAndAlreadyFired(unittest.TestCase):
    def test_double_fire_latches_degraded_first_value_stands(self):
        fsm = make_fsm()
        fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        fsm.handle_event(EVENT_PYRO1, 289.5, 18.55)
        decision = fsm.handle_event(EVENT_PYRO1, 150.0, 22.0)
        self.assertEqual(decision.action, Action.DEGRADED)
        self.assertEqual(decision.kind, "double_fire")
        self.assertTrue(fsm.degraded)
        self.assertEqual(fsm.pyro1, (18.55, 289.5))

    def test_refire_after_sequence_complete(self):
        fsm, _ = run_stream(nominal_events())
        for event_type in (EVENT_APOGEE, EVENT_PYRO1, EVENT_PYRO2):
            decision = fsm.handle_event(event_type, 50.0, 30.0)
            self.assertEqual(decision.action, Action.DEGRADED, event_type)
            self.assertEqual(decision.kind, "double_fire", event_type)
        self.assertEqual(fsm.apogee, (18.50, 290.0))
        self.assertEqual(fsm.pyro1, (18.55, 289.5))
        self.assertEqual(fsm.pyro2, (24.10, 98.0))


class TestMissingData(unittest.TestCase):
    def test_malformed_lines_rejected_state_unchanged(self):
        fsm = make_fsm()
        self.assertIsNone(fsm.handle_line("not an event line", 1.0))
        self.assertIsNone(fsm.handle_line("", 1.0))
        for bad in (
            "EVENT,PYRO1",
            "EVENT,PYRO1,abc",
            "EVENT,PYRO1,nan",
            "EVENT,PYRO1,inf",
            "EVENT",
        ):
            decision = fsm.handle_line(bad, 1.0)
            self.assertIsNotNone(decision, bad)
            self.assertEqual(decision.action, Action.REJECT, bad)
        self.assertEqual(fsm.state, State.WAIT_APOGEE)
        self.assertFalse(fsm.degraded)
        # The FSM keeps listening: a valid retry is still accepted.
        self.assertEqual(
            fsm.handle_line("EVENT,APOGEE,290.00", 1.0).action, Action.ACCEPT
        )


class TestAltitudeBounds(unittest.TestCase):
    def test_out_of_bounds_rejected(self):
        fsm = make_fsm()
        for alt in (-0.01, -100.0, MAX_ALT_M + 0.01, 1e9):
            decision = fsm.handle_event(EVENT_APOGEE, alt, 18.5)
            self.assertEqual(decision.action, Action.REJECT, alt)
            self.assertEqual(decision.kind, "altitude_out_of_bounds", alt)
        self.assertEqual(fsm.state, State.WAIT_APOGEE)
        boundary = fsm.handle_event(EVENT_APOGEE, MAX_ALT_M, 18.5)
        self.assertEqual(boundary.action, Action.ACCEPT)


class TestOrderingAndTimestamps(unittest.TestCase):
    def test_out_of_order_events_degraded(self):
        fsm = make_fsm()
        decision = fsm.handle_event(EVENT_PYRO1, 289.5, 18.5)
        self.assertEqual(decision.action, Action.DEGRADED)
        self.assertEqual(decision.kind, "out_of_order")
        self.assertEqual(fsm.state, State.WAIT_APOGEE)
        decision = fsm.handle_event(EVENT_PYRO2, 98.0, 19.0)
        self.assertEqual(decision.action, Action.DEGRADED)
        self.assertEqual(fsm.state, State.WAIT_APOGEE)
        self.assertTrue(fsm.degraded)

    def test_stale_timestamp_rejected(self):
        fsm = make_fsm()
        fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        stale = fsm.handle_event(EVENT_PYRO1, 289.5, 18.4)
        self.assertEqual(stale.action, Action.REJECT)
        self.assertEqual(stale.kind, "stale_timestamp")
        same_tick = fsm.handle_event(EVENT_PYRO1, 289.5, 18.5)
        self.assertEqual(same_tick.action, Action.ACCEPT)

    def test_unknown_event_type_rejected(self):
        fsm = make_fsm()
        decision = fsm.handle_event("BOOM", 100.0, 1.0)
        self.assertEqual(decision.action, Action.REJECT)
        self.assertEqual(decision.kind, "unknown_event")
        self.assertFalse(fsm.degraded)

    def test_pyro2_on_disabled_channel_degraded(self):
        fsm = make_fsm(pyro2_enabled=False)
        fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        fsm.handle_event(EVENT_PYRO1, 289.5, 18.55)
        decision = fsm.handle_event(EVENT_PYRO2, 98.0, 24.0)
        self.assertEqual(decision.action, Action.DEGRADED)
        self.assertEqual(decision.kind, "channel_disabled")
        self.assertIsNone(fsm.pyro2)

    def test_insufficient_separation_accepted_but_degraded(self):
        fsm = make_fsm()
        fsm.handle_event(EVENT_APOGEE, 290.0, 18.5)
        fsm.handle_event(EVENT_PYRO1, 289.5, 18.55)
        decision = fsm.handle_event(EVENT_PYRO2, 250.0, 19.0)
        self.assertEqual(decision.action, Action.ACCEPT)
        self.assertTrue(fsm.degraded)
        self.assertEqual(fsm.anomalies[-1]["kind"], "insufficient_separation")


class TestDeterminismAndRegression(unittest.TestCase):
    EXPECTED_ACTIONS = [
        Action.ACCEPT,       # jittered APOGEE cluster: first report commits
        Action.DUPLICATE, Action.DUPLICATE, Action.DUPLICATE,
        Action.DUPLICATE, Action.DUPLICATE,
        Action.DUPLICATE,    # exact retransmission (identical timestamp)
        Action.ACCEPT,       # PYRO1 fires
        Action.DUPLICATE,    # exact retransmission of PYRO1
        Action.DUPLICATE,    # retry inside dedup window
        Action.DEGRADED,     # genuine double fire of PYRO1
        Action.REJECT,       # PYRO2 altitude out of bounds
        Action.REJECT,       # PYRO2 stale timestamp
        Action.REJECT,       # unknown event type BOOM
        Action.ACCEPT,       # valid PYRO2 closes the sequence
    ]

    def test_pseudo_random_stream_regression_lock(self):
        fsm, decisions = run_stream(pseudo_random_stream())
        self.assertEqual([d.action for d in decisions], self.EXPECTED_ACTIONS)
        self.assertEqual(fsm.state, State.PYRO2_FIRED)
        self.assertTrue(fsm.degraded)
        kinds = [a["kind"] for a in fsm.anomalies]
        self.assertEqual(
            kinds,
            ["double_fire", "altitude_out_of_bounds", "stale_timestamp",
             "unknown_event"],
        )
        self.assertEqual(fsm.pyro1[0], 18.9)
        self.assertEqual(fsm.pyro2, (24.5, 98.0))

    def test_same_stream_repeated_runs_identical(self):
        results = []
        for _ in range(3):
            fsm, decisions = run_stream(pseudo_random_stream())
            results.append(
                (
                    [(d.action, d.kind, d.reason) for d in decisions],
                    fsm.state,
                    fsm.degraded,
                    fsm.anomalies,
                    fsm.apogee,
                    fsm.pyro1,
                    fsm.pyro2,
                )
            )
        self.assertEqual(results[0], results[1])
        self.assertEqual(results[1], results[2])

    def test_shuffled_seed_still_deterministic(self):
        for seed in (1, 7, 42, 999):
            first, _ = run_stream(pseudo_random_stream(seed))
            second, _ = run_stream(pseudo_random_stream(seed))
            self.assertEqual(first.anomalies, second.anomalies, seed)
            self.assertEqual(first.state, second.state, seed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
