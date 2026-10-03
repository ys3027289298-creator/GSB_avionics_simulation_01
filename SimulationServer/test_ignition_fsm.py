"""
Regression tests for the idempotent ignition state machine.

All event streams are constructed in-process from a seeded RNG, so the
tests need no rocketpy simulation, serial port, socket or telemetry server
and can be run as:

    python3 -m unittest test_ignition_fsm -v

The key regression guarantee: the same constructed event sequence must
produce exactly the same machine state on every run.
"""

import math
import random
import unittest

from ignition_fsm import (
    IgnitionStateMachine,
    ACCEPTED,
    DUPLICATE,
    REJECTED,
    RETRY,
    IGNORED,
)


NOMINAL_ALT = {"APOGEE": 900.0, "PYRO1": 899.0, "PYRO2": 100.0}
NOMINAL_TIME = {"APOGEE": 5.0, "PYRO1": 5.1, "PYRO2": 12.0}


def make_machine(**kwargs):
    defaults = dict(
        min_pyro_separation_s=2.0,
        max_altitude_m=1400.0,
        jitter_threshold_m=25.0,
        max_consecutive_parse_errors=5,
        enable_pyro2=True,
    )
    defaults.update(kwargs)
    return IgnitionStateMachine(**defaults)


def nominal_stream():
    return [(f"EVENT,{etype},{NOMINAL_ALT[etype]}", NOMINAL_TIME[etype])
            for etype in ("APOGEE", "PYRO1", "PYRO2")]


def replay(stream, **kwargs):
    fsm = make_machine(**kwargs)
    for line, sim_time in stream:
        fsm.process_line(line, sim_time)
    return fsm


def make_random_stream(seed, length=60):
    """Build a noisy anomaly-laden event stream from a seeded RNG.

    The anomalies mirror the ones the flight computer can produce:
    sensor jitter, identical-timestamp replays, duplicate ignitions,
    missing/garbage data and out-of-bounds altitudes.
    """
    rng = random.Random(seed)
    stream = []
    t = 5.0
    emitted = []
    malformed = ("EVENT,PYRO1,", "EVENT,,900.0", "EVENT", "garbage",
                 "EVENT,PYROX,1.0", "EVENT,PYRO1,not-a-number", "")

    for _ in range(length):
        roll = rng.random()
        if roll < 0.55:
            etype = rng.choice(("APOGEE", "PYRO1", "PYRO2"))
            alt = NOMINAL_ALT[etype] + rng.gauss(0.0, 3.0)
            stream.append((f"EVENT,{etype},{alt:.3f}", round(t, 3)))
            emitted.append((etype, t, alt))
        elif roll < 0.68:
            # Missing / corrupt data.
            stream.append((rng.choice(malformed), round(t, 3)))
        elif roll < 0.78:
            # Out-of-bounds altitude.
            etype = rng.choice(("APOGEE", "PYRO1", "PYRO2"))
            alt = rng.choice((-50.0, 50000.0, math.nan, math.inf))
            stream.append((f"EVENT,{etype},{alt}", round(t, 3)))
        elif roll < 0.90 and emitted:
            # Replay an already-emitted event at its original timestamp
            # (identical timestamp) or at a new one (duplicate ignition).
            etype, old_t, old_alt = rng.choice(emitted)
            if rng.random() < 0.5:
                stream.append((f"EVENT,{etype},{old_alt + rng.gauss(0.0, 40.0):.3f}",
                               round(old_t, 3)))
            else:
                stream.append((f"EVENT,{etype},{old_alt + rng.gauss(0.0, 40.0):.3f}",
                               round(t, 3)))
        else:
            # Non-event traffic, should be ignored.
            stream.append((f"STATUS,lock={rng.randint(0, 1)}", round(t, 3)))
        t += rng.uniform(0.05, 2.5)
    return stream


class NominalBehaviorTests(unittest.TestCase):
    def test_nominal_sequence_advances_to_pyro2(self):
        fsm = replay(nominal_stream())
        self.assertEqual(fsm.state, "PYRO2_FIRED")
        self.assertFalse(fsm.degraded)
        self.assertEqual(fsm.counts[ACCEPTED], 3)

    def test_non_event_lines_are_ignored(self):
        fsm = replay([("STATUS,ok", 1.0), ("HEARTBEAT", 2.0)] + nominal_stream())
        self.assertEqual(fsm.counts[IGNORED], 2)
        self.assertEqual(fsm.counts[ACCEPTED], 3)


class DuplicateIdempotencyTests(unittest.TestCase):
    def test_identical_timestamp_is_idempotent_noop(self):
        line, t = nominal_stream()[0]
        fsm = replay([(line, t), (line, t)] + nominal_stream()[1:])
        self.assertEqual(fsm.counts[DUPLICATE], 1)
        self.assertIsNotNone(fsm.first("APOGEE"))
        self.assertEqual(fsm.first("APOGEE")["sim_time"], t)

    def test_repeated_pyro1_fires_only_once(self):
        stream = nominal_stream()[:2] + [
            ("EVENT,PYRO1,880.0", 5.5),
            ("EVENT,PYRO1,870.0", 5.9),
        ] + nominal_stream()[2:]
        fsm = replay(stream)
        accepted = fsm.first("PYRO1")
        self.assertEqual(accepted["alt"], 899.0)
        self.assertEqual(fsm.counts[DUPLICATE], 2)
        self.assertEqual(fsm.state, "PYRO2_FIRED")

    def test_single_fire_invariant_for_all_channels(self):
        stream = nominal_stream()
        # Re-report every channel several times at new timestamps.
        stream += [("EVENT,APOGEE,800.0", 13.0),
                   ("EVENT,PYRO1,800.0", 13.5),
                   ("EVENT,PYRO2,90.0", 14.0)]
        fsm = replay(stream)
        for etype in ("APOGEE", "PYRO1", "PYRO2"):
            self.assertIsNotNone(fsm.first(etype))
        self.assertEqual(fsm.counts[ACCEPTED], 3)


class RejectTests(unittest.TestCase):
    def test_altitude_out_of_bounds_rejected(self):
        stream = [("EVENT,APOGEE,900.0", 5.0)]
        for i, alt in enumerate(("-5.0", "nan", "inf", "5000.0")):
            stream.append((f"EVENT,PYRO1,{alt}", 5.1 + i * 0.1))
        # Valid PYRO1 must still be accepted after the rejects.
        stream.append(("EVENT,PYRO1,899.0", 5.6))
        stream.append(("EVENT,PYRO2,100.0", 12.0))
        fsm = replay(stream)
        self.assertEqual(fsm.counts[REJECTED], 4)
        self.assertEqual(fsm.first("PYRO1")["alt"], 899.0)

    def test_pyro2_before_pyro1_rejected(self):
        fsm = replay([("EVENT,APOGEE,900.0", 5.0),
                      ("EVENT,PYRO2,100.0", 6.0)])
        self.assertIsNone(fsm.first("PYRO2"))
        self.assertEqual(fsm.counts[REJECTED], 1)

    def test_pyro2_below_separation_rejected_then_later_accepted(self):
        fsm = replay([("EVENT,APOGEE,900.0", 5.0),
                      ("EVENT,PYRO1,899.0", 5.1),
                      ("EVENT,PYRO2,500.0", 6.0),
                      ("EVENT,PYRO2,100.0", 12.0)])
        self.assertEqual(fsm.first("PYRO2")["alt"], 100.0)
        self.assertEqual(fsm.counts[REJECTED], 1)
        self.assertEqual(fsm.counts[ACCEPTED], 3)

    def test_pyro2_disarmed_rejected(self):
        fsm = replay(nominal_stream(), enable_pyro2=False)
        self.assertIsNone(fsm.first("PYRO2"))
        self.assertEqual(fsm.counts[REJECTED], 1)

    def test_non_monotonic_timestamp_rejected(self):
        fsm = replay([("EVENT,APOGEE,900.0", 5.0),
                      ("EVENT,PYRO1,899.0", 4.9),
                      ("EVENT,PYRO1,899.0", 5.1)])
        self.assertEqual(fsm.counts[REJECTED], 1)
        self.assertIsNotNone(fsm.first("PYRO1"))


class RetryAndDegradeTests(unittest.TestCase):
    def test_missing_data_retried_then_retransmission_accepted(self):
        stream = [nominal_stream()[0],
                  ("EVENT,PYRO1,", 5.1),
                  ("EVENT,,899.0", 5.1),
                  ("EVENT", 5.1),
                  ("EVENT,PYROX,899.0", 5.1)] + nominal_stream()[1:]
        fsm = replay(stream)
        self.assertEqual(fsm.counts[RETRY], 4)
        self.assertFalse(fsm.degraded)
        self.assertIsNotNone(fsm.first("APOGEE"))
        self.assertIsNotNone(fsm.first("PYRO1"))

    def test_retry_storm_enters_degraded_mode(self):
        fsm = replay([("EVENT,PYRO1,", 5.1 + 0.1 * i) for i in range(6)])
        self.assertTrue(fsm.degraded)
        self.assertEqual(fsm.counts[RETRY], 6)
        self.assertIsNone(fsm.first("PYRO1"))

    def test_sensor_jitter_on_repeated_detection_degrades(self):
        fsm = replay([("EVENT,APOGEE,900.0", 5.0),
                      ("EVENT,APOGEE,815.0", 5.4)] + nominal_stream()[1:])
        self.assertTrue(fsm.degraded)
        self.assertEqual(fsm.first("APOGEE")["alt"], 900.0)
        self.assertEqual(fsm.counts[DUPLICATE], 1)

    def test_small_jitter_does_not_degrade(self):
        fsm = replay([("EVENT,APOGEE,900.0", 5.0),
                      ("EVENT,APOGEE,905.0", 5.4)] + nominal_stream()[1:])
        self.assertFalse(fsm.degraded)

    def test_pyro1_without_apogee_degrades(self):
        fsm = replay([("EVENT,PYRO1,899.0", 5.1),
                      ("EVENT,PYRO2,100.0", 12.0)])
        self.assertTrue(fsm.degraded)
        self.assertEqual(fsm.state, "PYRO2_FIRED")


class DeterminismRegressionTests(unittest.TestCase):
    """Same constructed sequence must always yield the same outcome."""

    def test_random_stream_replay_is_identical(self):
        for seed in range(50):
            stream = make_random_stream(seed)
            first = replay(stream).summary()
            second = replay(stream).summary()
            self.assertEqual(first, second, f"non-deterministic for seed={seed}")

    def test_random_stream_single_fire_invariant(self):
        for seed in range(50):
            summary = replay(make_random_stream(seed)).summary()
            for etype in ("APOGEE", "PYRO1", "PYRO2"):
                accepted = [a for a in (summary["accepted"][etype],) if a]
                self.assertLessEqual(len(accepted), 1)

    def test_random_streams_exercise_every_anomaly_class(self):
        kinds = {ACCEPTED, DUPLICATE, REJECTED, RETRY, IGNORED}
        seen = set()
        for seed in range(50):
            summary = replay(make_random_stream(seed)).summary()
            for kind in kinds:
                if summary["counts"][kind]:
                    seen.add(kind)
        self.assertEqual(seen, kinds)

    def test_degraded_outcome_stable_across_runs(self):
        degraded_seeds = []
        for seed in range(50):
            stream = make_random_stream(seed)
            r1 = replay(stream).summary()
            r2 = replay(stream).summary()
            self.assertEqual(r1["degraded"], r2["degraded"])
            self.assertEqual(r1["degraded_reasons"], r2["degraded_reasons"])
            if r1["degraded"]:
                degraded_seeds.append(seed)
        self.assertTrue(degraded_seeds,
                        "random corpus should contain degraded cases")


if __name__ == "__main__":
    unittest.main(verbosity=2)
