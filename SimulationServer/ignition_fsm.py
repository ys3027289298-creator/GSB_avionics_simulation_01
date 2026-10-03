"""
Ignition state machine with idempotency guarantees.

The flight computer streams telemetry frames to the simulation server and
reports detected events as `EVENT,<TYPE>,<ALTITUDE>` lines.  The whole
ignition sequence is modelled as a linear state machine:

    IDLE -> APOGEE_DETECTED -> PYRO1_FIRED -> PYRO2_FIRED

plus an orthogonal "degraded" flag that records sensor / link problems while
the sequence keeps progressing.

Anomaly policy (see ANOMALY_POLICY.md):

* REJECT    - invalid event that must never be acted on: altitude out of
               bounds, timestamp going backwards, PYRO2 before PYRO1 or
               closer than the minimum separation, PYRO2 while the channel
               is disabled.
* DUPLICATE - idempotent replay of an event already accepted: same
               (type, timestamp), or a channel that has already fired.  The
               first event wins, state does not change.
* RETRY     - missing / unparseable data: the state machine stays put and
               waits for a retransmission.  Too many consecutive parse
               failures pushes the machine into degraded mode.
* DEGRADED  - the sequence keeps running but the event stream is suspicious:
               repeated detection whose altitude disagrees beyond the jitter
               threshold, PYRO1 without a preceding APOGEE, or a retry storm.

The machine is a pure function of the event stream: it performs no I/O and
uses no global randomness, so feeding the same sequence twice always yields
the same result.
"""

import math


# --- Event types (must match the wire protocol: EVENT,<TYPE>,<ALT>) ---------

APOGEE = "APOGEE"
PYRO1 = "PYRO1"
PYRO2 = "PYRO2"
KNOWN_TYPES = (APOGEE, PYRO1, PYRO2)


# --- Outcomes returned by the machine for every processed line -------------

ACCEPTED = "accepted"    # event validated and advanced the state machine
DUPLICATE = "duplicate"  # idempotent no-op, an equivalent event won already
REJECTED = "rejected"    # permanently invalid event, never acted on
RETRY = "retry"          # missing/unparseable data, awaiting retransmission
IGNORED = "ignored"      # line that is not an EVENT report (other traffic)


# Main progression states.
IDLE = "IDLE"
APOGEE_DETECTED = "APOGEE_DETECTED"
PYRO1_FIRED = "PYRO1_FIRED"
PYRO2_FIRED = "PYRO2_FIRED"

_RANK = {IDLE: 0, APOGEE_DETECTED: 1, PYRO1_FIRED: 2, PYRO2_FIRED: 3}
_RANK_AFTER = {APOGEE: APOGEE_DETECTED, PYRO1: PYRO1_FIRED, PYRO2: PYRO2_FIRED}


class IgnitionStateMachine:
    """Stateful validator for incoming `EVENT,...` reports.

    Parameters
    ----------
    min_pyro_separation_s:
        Minimum time between PYRO1 and PYRO2.
    max_altitude_m / min_altitude_m:
        Valid altitude envelope for a reported event.
    jitter_threshold_m:
        Altitude disagreement on repeated detections that is attributed to
        sensor jitter and flips the machine into degraded mode.
    max_consecutive_parse_errors:
        Number of consecutive unparseable/missing events tolerated before
        the link is considered degraded.
    enable_pyro2:
        Whether the PYRO2 channel is armed.
    timestamp_resolution:
        Timestamps are quantised to this precision (seconds) before being
        used as idempotency keys, matching the server's `round(t, 3)`.
    """

    def __init__(
        self,
        min_pyro_separation_s=2.0,
        max_altitude_m=math.inf,
        min_altitude_m=0.0,
        jitter_threshold_m=25.0,
        max_consecutive_parse_errors=5,
        enable_pyro2=True,
        timestamp_resolution=3,
    ):
        self.min_pyro_separation_s = min_pyro_separation_s
        self.max_altitude_m = max_altitude_m
        self.min_altitude_m = min_altitude_m
        self.jitter_threshold_m = jitter_threshold_m
        self.max_consecutive_parse_errors = max_consecutive_parse_errors
        self.enable_pyro2 = enable_pyro2
        self.timestamp_resolution = timestamp_resolution

        self.state = IDLE
        self.degraded = False
        self.degraded_reasons = []

        # At most one accepted event per channel; first one wins.
        self.accepted = {APOGEE: None, PYRO1: None, PYRO2: None}

        self.counts = {
            ACCEPTED: 0,
            DUPLICATE: 0,
            REJECTED: 0,
            RETRY: 0,
            IGNORED: 0,
        }
        self.anomalies = []

        self._seen_keys = set()
        self._last_time = None
        self._consecutive_parse_errors = 0

    # -- public API ---------------------------------------------------------

    def process_line(self, line, sim_time):
        """Process one raw line received from the flight computer.

        Returns `(outcome, payload)` where payload is the normalised event
        dict `{"type", "sim_time", "alt"}` for ACCEPTED, an anomaly dict
        for DUPLICATE/REJECTED/RETRY, or `None` for IGNORED.
        """
        if line is None:
            return self._retry(sim_time, None, "empty line", None)

        parts = [p.strip() for p in str(line).split(",")]
        if not parts or parts[0] != "EVENT":
            self.counts[IGNORED] += 1
            return IGNORED, None

        if len(parts) < 3 or not parts[1] or parts[2] == "":
            return self._retry(sim_time, parts[1] if len(parts) > 1 else None,
                               "missing fields", line)

        event_type = parts[1]
        if event_type not in KNOWN_TYPES:
            return self._retry(sim_time, event_type, "unknown event type", line)

        try:
            altitude = float(parts[2])
        except ValueError:
            return self._retry(sim_time, event_type, "non-numeric altitude", line)

        return self.process_event(event_type, sim_time, altitude)

    def process_event(self, event_type, sim_time, altitude):
        """Process one fully parsed event.  Pure validation step."""
        event = {
            "type": event_type,
            "sim_time": round(float(sim_time), self.timestamp_resolution),
            "alt": float(altitude),
        }
        sim_time = event["sim_time"]

        # Idempotency key: the exact same report replayed (same timestamp)
        # must never be acted on twice.
        key = (event_type, sim_time)
        if key in self._seen_keys:
            return self._duplicate(event, "identical timestamp replay")

        # A channel that has already fired: first fire wins, always.
        if self.accepted[event_type] is not None:
            first = self.accepted[event_type]
            if abs(event["alt"] - first["alt"]) > self.jitter_threshold_m:
                self._degrade(
                    f"{event_type} re-reported with jittering altitude "
                    f"({first['alt']:.2f}m -> {event['alt']:.2f}m)"
                )
            return self._duplicate(event, f"{event_type} already fired/detected")

        # Altitude envelope.  NaN / inf never pass.
        if not math.isfinite(altitude) or altitude < self.min_altitude_m \
                or altitude > self.max_altitude_m:
            return self._reject(event, "altitude out of bounds")

        # Monotonic timestamps (equal timestamps were handled above).
        if self._last_time is not None and sim_time < self._last_time:
            return self._reject(event, "non-monotonic timestamp")

        # Ordering and arming constraints.
        if event_type == PYRO2 and not self.enable_pyro2:
            return self._reject(event, "PYRO2 channel not armed")
        if event_type == PYRO2 and self.accepted[PYRO1] is None:
            return self._reject(event, "PYRO2 before PYRO1")
        if event_type == PYRO2:
            gap = sim_time - self.accepted[PYRO1]["sim_time"]
            if gap < self.min_pyro_separation_s:
                return self._reject(
                    event,
                    f"PYRO2 too soon after PYRO1 (gap={gap:.3f}s)",
                )
        if event_type == PYRO1 and self.accepted[APOGEE] is None:
            # The APOGEE report was lost; keep the ignition safe but flag it.
            self._degrade("PYRO1 without preceding APOGEE report")

        # Accept: advance the machine.
        self._seen_keys.add(key)
        self.accepted[event_type] = event
        self.counts[ACCEPTED] += 1
        self._last_time = sim_time
        self._consecutive_parse_errors = 0

        new_state = _RANK_AFTER[event_type]
        if _RANK[new_state] > _RANK[self.state]:
            self.state = new_state
        return ACCEPTED, event

    def first(self, event_type):
        """The single accepted event for a channel, or None."""
        return self.accepted[event_type]

    def summary(self):
        """Deterministic snapshot used for session logging / test reports."""
        return {
            "state": self.state,
            "degraded": self.degraded,
            "degraded_reasons": list(self.degraded_reasons),
            "counts": dict(self.counts),
            "accepted": {
                event_type: (dict(event) if event else None)
                for event_type, event in self.accepted.items()
            },
            "anomalies": [dict(item) for item in self.anomalies],
        }

    # -- internal helpers ---------------------------------------------------

    def _record_anomaly(self, event, kind, reason):
        alt = event.get("alt") if isinstance(event, dict) else None
        if isinstance(alt, float) and not math.isfinite(alt):
            # NaN/Inf must not leak into JSON logs or break equality checks.
            alt = None
        record = {
            "kind": kind,
            "reason": reason,
            "type": event.get("type") if isinstance(event, dict) else None,
            "sim_time": event.get("sim_time") if isinstance(event, dict) else None,
            "alt": alt,
        }
        self.anomalies.append(record)
        return record

    def _duplicate(self, event, reason):
        self.counts[DUPLICATE] += 1
        record = self._record_anomaly(event, DUPLICATE, reason)
        return DUPLICATE, record

    def _reject(self, event, reason):
        self.counts[REJECTED] += 1
        record = self._record_anomaly(event, REJECTED, reason)
        return REJECTED, record

    def _retry(self, sim_time, event_type, reason, raw_line):
        self.counts[RETRY] += 1
        self._consecutive_parse_errors += 1
        event = {
            "type": event_type,
            "sim_time": (round(float(sim_time), self.timestamp_resolution)
                         if sim_time is not None else None),
            "alt": None,
        }
        record = self._record_anomaly(event, RETRY, reason)
        record["raw_line"] = raw_line
        if self._consecutive_parse_errors >= self.max_consecutive_parse_errors:
            self._degrade(
                f"{self._consecutive_parse_errors} consecutive unparseable/"
                f"missing events (last: {reason})"
            )
        return RETRY, record

    def _degrade(self, reason):
        self.degraded = True
        if reason not in self.degraded_reasons:
            self.degraded_reasons.append(reason)
