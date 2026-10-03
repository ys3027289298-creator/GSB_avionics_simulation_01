"""Idempotent ignition state machine for the simulation server.

The server streams telemetry frames to the firmware under test and receives
``EVENT,<TYPE>,<altitude>`` lines back, where TYPE is one of APOGEE, PYRO1 or
PYRO2. This module decides, deterministically, what every inbound event means.

Ignition sequence states:

    WAIT_APOGEE --APOGEE--> APOGEE_CONFIRMED --PYRO1--> PYRO1_FIRED --PYRO2--> PYRO2_FIRED

Every event is classified into one of four dispositions:

- ACCEPT    valid event; the state machine advances and the event is committed.
- DUPLICATE idempotent redelivery (identical retransmission, or a retry inside
            the dedup window caused by sensor jitter). No state change, not an
            error. This is the "retry-safe" disposition.
- REJECT    malformed or physically impossible input: missing fields,
            non-numeric / NaN / infinite altitude, altitude out of bounds,
            stale timestamp, unknown event type. The event is dropped, the
            state is unchanged, and the server keeps listening, so the
            firmware may retry with a valid event.
- DEGRADED  a genuine flight-logic anomaly: double fire of a channel,
            out-of-order event (PYRO1 before APOGEE, PYRO2 before PYRO1),
            event on a disabled channel, or an event after sequence
            completion. The event is not committed, the *first* committed
            value stands, and the degraded latch is set so the session is
            reported as degraded. Recording continues.

Idempotency:

- Key 1 (exact): (event_type, timestamp rounded to ms, altitude rounded to
  cm). Identical keys are always answered DUPLICATE, even if replayed late.
- Key 2 (dedup window): same event type within ``dedup_window_s`` and
  ``dedup_alt_tolerance_m`` of the committed value is a tolerated retry.
  Anything beyond those tolerances after the channel was already committed
  is a double fire and latches DEGRADED.

Determinism: the FSM is a pure function of the inbound event sequence. The
same sequence run through freshly constructed FSMs always yields identical
decisions, final state, committed values and anomaly records.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Set, Tuple

EVENT_APOGEE = "APOGEE"
EVENT_PYRO1 = "PYRO1"
EVENT_PYRO2 = "PYRO2"
KNOWN_EVENTS = frozenset((EVENT_APOGEE, EVENT_PYRO1, EVENT_PYRO2))

TIME_EPS_S = 1e-9


class State(Enum):
    WAIT_APOGEE = "WAIT_APOGEE"
    APOGEE_CONFIRMED = "APOGEE_CONFIRMED"
    PYRO1_FIRED = "PYRO1_FIRED"
    PYRO2_FIRED = "PYRO2_FIRED"


class Action(Enum):
    ACCEPT = "accept"
    DUPLICATE = "duplicate"
    REJECT = "reject"
    DEGRADED = "degraded"


@dataclass(frozen=True)
class Decision:
    action: Action
    kind: str
    reason: str
    event_type: Optional[str] = None
    alt: Optional[float] = None


_NEXT_STATE: Dict[Tuple[State, str], State] = {
    (State.WAIT_APOGEE, EVENT_APOGEE): State.APOGEE_CONFIRMED,
    (State.APOGEE_CONFIRMED, EVENT_PYRO1): State.PYRO1_FIRED,
    (State.PYRO1_FIRED, EVENT_PYRO2): State.PYRO2_FIRED,
}


class IgnitionFSM:
    """State machine validating and recording the ignition event sequence."""

    def __init__(
        self,
        max_alt_m: float,
        dedup_window_s: float = 0.5,
        dedup_alt_tolerance_m: float = 5.0,
        min_pyro_separation_s: float = 0.0,
        pyro2_enabled: bool = True,
    ):
        self.max_alt_m = max_alt_m
        self.dedup_window_s = dedup_window_s
        self.dedup_alt_tolerance_m = dedup_alt_tolerance_m
        self.min_pyro_separation_s = min_pyro_separation_s
        self.pyro2_enabled = pyro2_enabled

        self.state = State.WAIT_APOGEE
        self.degraded = False
        self.anomalies: List[dict] = []
        self._committed: Dict[str, Tuple[float, float]] = {}
        self._seen_keys: Set[Tuple[str, float, float]] = set()
        self._clock: Optional[float] = None

    # -- committed events (first valid value per type) ----------------------

    @property
    def apogee(self) -> Optional[Tuple[float, float]]:
        return self._committed.get(EVENT_APOGEE)

    @property
    def pyro1(self) -> Optional[Tuple[float, float]]:
        return self._committed.get(EVENT_PYRO1)

    @property
    def pyro2(self) -> Optional[Tuple[float, float]]:
        return self._committed.get(EVENT_PYRO2)

    # -- entry points --------------------------------------------------------

    def handle_line(self, line: str, sim_time: float) -> Optional[Decision]:
        """Parse one wire line ``EVENT,<TYPE>,<alt>`` and process it.

        Returns None for non-EVENT lines (they are ignored), otherwise a
        Decision. The wire format and field names are not modified here.
        """
        parts = [p.strip() for p in line.split(",")]
        if not parts or parts[0] != "EVENT":
            return None
        event_type = parts[1] if len(parts) > 1 else None
        if len(parts) < 3:
            return self._reject(
                "missing_fields",
                "malformed EVENT line: missing fields",
                event_type,
                None,
                sim_time,
            )
        try:
            alt = float(parts[2])
        except ValueError:
            return self._reject(
                "non_numeric_alt",
                f"non-numeric altitude {parts[2]!r}",
                event_type,
                None,
                sim_time,
            )
        return self.handle_event(event_type, alt, sim_time)

    def handle_event(self, event_type: str, alt: float, sim_time: float) -> Decision:
        """Classify one parsed event. Pure: depends only on FSM state + args."""
        # 1. Protocol-level validity ----------------------------------------
        if event_type not in KNOWN_EVENTS:
            return self._reject(
                "unknown_event",
                f"unknown event type {event_type!r}",
                event_type,
                alt,
                sim_time,
            )
        if not math.isfinite(alt):
            return self._reject(
                "non_finite_alt",
                f"non-finite altitude {alt}",
                event_type,
                alt,
                sim_time,
            )
        if alt < 0.0 or alt > self.max_alt_m:
            return self._reject(
                "altitude_out_of_bounds",
                f"altitude {alt:.2f}m outside [0, {self.max_alt_m:.2f}m]",
                event_type,
                alt,
                sim_time,
            )

        # 2. Idempotency key — exact replays are always harmless -------------
        key = (event_type, round(sim_time, 3), round(alt, 2))
        if key in self._seen_keys:
            return Decision(
                Action.DUPLICATE,
                "exact_retransmission",
                f"exact retransmission of {event_type}",
                event_type,
                alt,
            )

        # 3. Timestamp monotonicity (equal timestamps are allowed) ----------
        if self._clock is not None and sim_time < self._clock - TIME_EPS_S:
            return self._reject(
                "stale_timestamp",
                f"timestamp {sim_time:.3f}s before last accepted "
                f"{self._clock:.3f}s",
                event_type,
                alt,
                sim_time,
            )
        self._seen_keys.add(key)
        self._clock = sim_time if self._clock is None else max(self._clock, sim_time)

        # 4. Channel already committed — retry vs double fire ---------------
        committed = self._committed.get(event_type)
        if committed is not None:
            committed_t, committed_alt = committed
            if (
                abs(sim_time - committed_t) <= self.dedup_window_s
                and abs(alt - committed_alt) <= self.dedup_alt_tolerance_m
            ):
                return Decision(
                    Action.DUPLICATE,
                    "retry_in_dedup_window",
                    f"retry of {event_type} within dedup window",
                    event_type,
                    alt,
                )
            return self._degrade(
                "double_fire",
                f"conflicting repeat of {event_type}: first committed at "
                f"T+{committed_t:.3f}s alt={committed_alt:.2f}m, repeat at "
                f"T+{sim_time:.3f}s alt={alt:.2f}m",
                event_type,
                alt,
                sim_time,
            )

        # 5. Configuration and ordering constraints -------------------------
        if event_type == EVENT_PYRO2 and not self.pyro2_enabled:
            return self._degrade(
                "channel_disabled",
                "PYRO2 received but channel is disabled",
                event_type,
                alt,
                sim_time,
            )

        next_state = _NEXT_STATE.get((self.state, event_type))
        if next_state is None:
            return self._degrade(
                "out_of_order",
                f"{event_type} not expected in state {self.state.value}",
                event_type,
                alt,
                sim_time,
            )

        # 6. Commit ----------------------------------------------------------
        self._committed[event_type] = (sim_time, alt)
        self.state = next_state

        if (
            event_type == EVENT_PYRO2
            and self.min_pyro_separation_s > 0.0
            and EVENT_PYRO1 in self._committed
            and sim_time - self._committed[EVENT_PYRO1][0]
            < self.min_pyro_separation_s
        ):
            gap = sim_time - self._committed[EVENT_PYRO1][0]
            self._latch(
                "insufficient_separation",
                f"PYRO1->PYRO2 gap {gap:.3f}s below minimum "
                f"{self.min_pyro_separation_s}s",
                event_type,
                alt,
                sim_time,
            )

        return Decision(Action.ACCEPT, "valid", f"{event_type} accepted", event_type, alt)

    # -- bookkeeping ---------------------------------------------------------

    def _record(self, action, kind, reason, event_type, alt, sim_time):
        self.anomalies.append(
            {
                "sim_time": round(sim_time, 3),
                "action": action.value,
                "kind": kind,
                "type": event_type,
                "alt": alt,
                "reason": reason,
            }
        )

    def _latch(self, kind, reason, event_type, alt, sim_time):
        self.degraded = True
        self._record(Action.DEGRADED, kind, reason, event_type, alt, sim_time)

    def _reject(self, kind, reason, event_type, alt, sim_time):
        self._record(Action.REJECT, kind, reason, event_type, alt, sim_time)
        return Decision(Action.REJECT, kind, reason, event_type, alt)

    def _degrade(self, kind, reason, event_type, alt, sim_time):
        self._latch(kind, reason, event_type, alt, sim_time)
        return Decision(Action.DEGRADED, kind, reason, event_type, alt)
