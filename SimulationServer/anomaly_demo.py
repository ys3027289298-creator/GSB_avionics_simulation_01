"""Reproduce ignition anomalies with constructed event streams.

No RocketPy, no serial ports, no sockets, no external services — every stream
below is built locally and fed straight into the IgnitionFSM, exactly the way
run_session() in server.py does. Run:

    python3 anomaly_demo.py
"""

from ignition_fsm import Action, IgnitionFSM

MAX_ALT_M = 450.0


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


def show(title, fsm, lines):
    print(f"\n=== {title} ===")
    for sim_time, line in lines:
        decision = fsm.handle_line(line, sim_time)
        if decision is None:
            print(f"  T+{sim_time:6.2f}s  {line!r:32} -> (ignored, not an EVENT line)")
        else:
            print(f"  T+{sim_time:6.2f}s  {line!r:32} -> "
                  f"{decision.action.value.upper():9} [{decision.kind}] {decision.reason}")
    print(f"  final state={fsm.state.value}  degraded={fsm.degraded}")


def main():
    show("nominal dual deploy", make_fsm(), [
        (18.50, "EVENT,APOGEE,290.00"),
        (18.55, "EVENT,PYRO1,289.50"),
        (24.10, "EVENT,PYRO2,98.00"),
    ])

    show("identical timestamps (idempotent replay)", make_fsm(), [
        (18.50, "EVENT,APOGEE,290.00"),
        (18.50, "EVENT,APOGEE,290.00"),
        (18.50, "EVENT,PYRO1,289.50"),
        (18.50, "EVENT,PYRO1,289.50"),
    ])

    show("sensor jitter around apogee", make_fsm(), [
        (18.50, "EVENT,APOGEE,290.00"),
        (18.55, "EVENT,APOGEE,290.31"),
        (18.60, "EVENT,APOGEE,289.72"),
        (18.65, "EVENT,APOGEE,290.18"),
    ])

    show("duplicate fire vs already fired", make_fsm(), [
        (18.50, "EVENT,APOGEE,290.00"),
        (18.55, "EVENT,PYRO1,289.50"),
        (18.70, "EVENT,PYRO1,289.40"),   # retry inside dedup window
        (22.00, "EVENT,PYRO1,150.00"),   # genuine double fire -> degraded
        (24.10, "EVENT,PYRO2,98.00"),
        (30.00, "EVENT,PYRO2,95.00"),    # already fired -> degraded
    ])

    show("missing / malformed data", make_fsm(), [
        (1.00, "EVENT,PYRO1"),
        (1.01, "EVENT,PYRO1,abc"),
        (1.02, "EVENT,PYRO1,nan"),
        (1.03, "EVENT"),
        (1.04, "EVENT,APOGEE,290.00"),   # valid retry still accepted
    ])

    show("altitude out of bounds", make_fsm(), [
        (18.50, "EVENT,APOGEE,-3.00"),
        (18.50, "EVENT,APOGEE,99999.00"),
        (18.50, "EVENT,APOGEE,290.00"),
    ])

    show("out of order + stale timestamp", make_fsm(), [
        (18.50, "EVENT,PYRO2,98.00"),    # PYRO2 before anything -> degraded
        (18.60, "EVENT,APOGEE,290.00"),
        (18.55, "EVENT,PYRO1,289.50"),   # stale timestamp -> reject
        (18.70, "EVENT,PYRO1,289.50"),   # valid retry -> accept
    ])


if __name__ == "__main__":
    main()
