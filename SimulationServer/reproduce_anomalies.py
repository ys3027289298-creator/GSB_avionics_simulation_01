"""
Reproduce every ignition anomaly using constructed event streams.

No rocketpy simulation, serial port, socket or telemetry server is involved:
each stream is a hand-built list of (raw_line, sim_time) pairs fed straight
into IgnitionStateMachine.  Run:

    python3 reproduce_anomalies.py
"""

import math

from ignition_fsm import (
    IgnitionStateMachine,
    ACCEPTED,
    DUPLICATE,
    REJECTED,
    RETRY,
    IGNORED,
)


def run_case(title, events, **fsm_kwargs):
    print(f"\n--- {title} ---")
    fsm = IgnitionStateMachine(**fsm_kwargs)
    for line, sim_time in events:
        outcome, payload = fsm.process_line(line, sim_time)
        detail = ""
        if outcome in (REJECTED, RETRY, DUPLICATE):
            detail = f"  -> {payload['reason']}"
        print(f"  t={sim_time:>5}  {line!r:<28} => {outcome}{detail}")
    summary = fsm.summary()
    print(f"  state={summary['state']} degraded={summary['degraded']} "
          f"counts={summary['counts']}")
    if summary["degraded_reasons"]:
        print(f"  degraded_reasons={summary['degraded_reasons']}")
    return fsm


nominal = [
    ("EVENT,APOGEE,900.0", 5.0),
    ("EVENT,PYRO1,899.0", 5.1),
    ("EVENT,PYRO2,100.0", 12.0),
]


def main():
    # Baseline: clean sequence reaches PYRO2_FIRED without anomalies.
    run_case("Nominal sequence", nominal)

    # 1. Identical timestamp replay: second copy must be an idempotent no-op.
    run_case("Identical timestamp (APOGEE replayed at the same t)",
             [("EVENT,APOGEE,900.0", 5.0),
              ("EVENT,APOGEE,900.0", 5.0)] + nominal[1:])

    # 2. Sensor jitter: repeated APOGEE at a wildly different altitude.
    run_case("Sensor jitter on repeated APOGEE",
             [("EVENT,APOGEE,900.0", 5.0),
              ("EVENT,APOGEE,815.0", 5.4)] + nominal[1:])

    # 3/4. Duplicate ignition / channel already fired.
    run_case("Repeated PYRO1 after it already fired",
             nominal[:2] + [("EVENT,PYRO1,880.0", 5.5)] + nominal[2:])

    # 5. Missing data: empty altitude, missing fields, garbage, unknown type.
    run_case("Missing / unparseable data (then retransmission recovers)",
             [("EVENT,PYRO1,", 5.1),
              ("EVENT,,899.0", 5.1),
              ("EVENT", 5.1),
              ("garbage line", 5.1),
              ("EVENT,PYROX,899.0", 5.1)] + nominal[1:])
    run_case("Retry storm beyond threshold -> degraded",
             [("EVENT,PYRO1,", 5.1 + i * 0.1) for i in range(6)])

    # 6. Altitude out of bounds (negative / NaN / infinite / way too high).
    run_case("Altitude out of bounds",
             [("EVENT,APOGEE,900.0", 5.0),
              ("EVENT,PYRO1,-5.0", 5.1),
              ("EVENT,PYRO1,nan", 5.2),
              ("EVENT,PYRO1,inf", 5.3),
              ("EVENT,PYRO1,99999.0", 5.4)] + nominal[1:],
             max_altitude_m=1400.0)

    # Ordering / separation violations.
    run_case("PYRO2 reported before PYRO1",
             [("EVENT,APOGEE,900.0", 5.0),
              ("EVENT,PYRO2,100.0", 6.0)])
    run_case("PYRO2 closer than min separation",
             [("EVENT,APOGEE,900.0", 5.0),
              ("EVENT,PYRO1,899.0", 5.1),
              ("EVENT,PYRO2,500.0", 6.0)])
    run_case("PYRO2 channel disarmed",
             nominal, enable_pyro2=False)

    # Timestamp regression.
    run_case("Non-monotonic timestamp",
             [("EVENT,APOGEE,900.0", 5.0),
              ("EVENT,PYRO1,899.0", 4.9)])

    print("\nAll anomaly streams reproduced deterministically.")


if __name__ == "__main__":
    main()
