import argparse
import serial
import serial.tools.list_ports
import socket
import time
import math
import random
import json
import os
import requests
from datetime import datetime
from rocketpy import Environment, Rocket, SolidMotor, Flight

# --- Load config -----------------------------------------------------------
parser = argparse.ArgumentParser()
parser.add_argument("--config", default="config.json", help="Path to config file")
args = parser.parse_args()

with open(args.config) as f:
    cfg = json.load(f)

env_cfg   = cfg["environment"]
motor_cfg = cfg["motor"]
rocket_cfg = cfg["rocket"]
flight_cfg = cfg["flight"]
noise_cfg  = cfg["sensor_noise"]
sim_cfg    = cfg["sim"]
val_cfg    = cfg["validation"]

# --- Derived constants -----------------------------------------------------
BAUD_RATE             = 115200
TCP_PORT              = sim_cfg["socket_port"]
DT                    = sim_cfg["dt"]
REALTIME              = True
LOG_FILE              = "sessions.json"

PRESSURE_NOISE_STD    = noise_cfg["pressure_std"]
BARO_NOISE_STD        = noise_cfg["baro_std"]
ACCEL_NOISE_STD       = noise_cfg["accel_std_g"] * 9.81 * math.sqrt(100)
GYRO_NOISE_STD        = math.radians(noise_cfg["gyro_std_deg"]) * math.sqrt(100)
GPS_NOISE_STD_DEG     = noise_cfg["gps_std_meters"] / 111320

MIN_PYRO_SEPARATION_S = val_cfg["min_pyro_separation_s"]


# ---------------------------------------------------------------
# Flight setup
# ---------------------------------------------------------------

def build_flight():
    print("[SIM] Setting up flight...")

    env = Environment(
        latitude=env_cfg["latitude"],
        longitude=env_cfg["longitude"],
        elevation=env_cfg["elevation"],
    )

    motor = SolidMotor(
        thrust_source=motor_cfg["thrust_source"],
        dry_mass=motor_cfg["dry_mass"],
        dry_inertia=tuple(motor_cfg["dry_inertia"]),
        nozzle_radius=motor_cfg["nozzle_radius"],
        grain_number=motor_cfg["grain_number"],
        grain_density=motor_cfg["grain_density"],
        grain_outer_radius=motor_cfg["grain_outer_radius"],
        grain_initial_inner_radius=motor_cfg["grain_initial_inner_radius"],
        grain_initial_height=motor_cfg["grain_initial_height"],
        grain_separation=motor_cfg["grain_separation"],
        grains_center_of_mass_position=motor_cfg["grains_center_of_mass_position"],
        center_of_dry_mass_position=motor_cfg["center_of_dry_mass_position"],
        nozzle_position=motor_cfg["nozzle_position"],
        burn_time=motor_cfg["burn_time"],
        throat_radius=motor_cfg["throat_radius"],
    )

    rocket = Rocket(
        radius=rocket_cfg["radius"],
        mass=rocket_cfg["mass"],
        inertia=tuple(rocket_cfg["inertia"]),
        power_off_drag=rocket_cfg["power_off_drag"],
        power_on_drag=rocket_cfg["power_on_drag"],
        center_of_mass_without_motor=rocket_cfg["center_of_mass_without_motor"],
        coordinate_system_orientation="tail_to_nose",
    )
    rocket.add_motor(motor, position=0)
    rocket.add_nose(**rocket_cfg["nose"])
    rocket.add_fins(**rocket_cfg["fins"])
    rocket.add_parachute(**rocket_cfg["parachute"])

    flight = Flight(
        rocket=rocket, environment=env,
        rail_length=flight_cfg["rail_length"],
        inclination=flight_cfg["inclination"],
        heading=flight_cfg["heading"],
        terminate_on_apogee=False,
    )

    print(f"[SIM] Apogee: {flight.apogee - env.elevation:.1f}m AGL "
          f"at T+{flight.apogee_time:.2f}s")
    return flight, env


# ---------------------------------------------------------------
# Transport abstraction
# ---------------------------------------------------------------

class SerialTransport:
    def __init__(self, port, baud):
        self.ser  = serial.Serial(port, baud, timeout=1)
        self.mode = "serial"
        time.sleep(2)
        self.ser.reset_input_buffer()

    def write(self, data: bytes):
        self.ser.write(data)

    def readline(self) -> str:
        if self.ser.in_waiting:
            return self.ser.readline().decode().strip()
        return ""

    def close(self):
        self.ser.close()


class TcpTransport:
    def __init__(self, conn):
        self.conn = conn
        self.conn.setblocking(False)
        self.buf  = ""
        self.mode = "tcp"

    def write(self, data: bytes):
        self.conn.sendall(data)

    def readline(self) -> str:
        try:
            chunk = self.conn.recv(1024).decode()
            self.buf += chunk
        except BlockingIOError:
            pass
        if "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            return line.strip()
        return ""

    def close(self):
        self.conn.close()


# ---------------------------------------------------------------
# Connection — first come first served
# ---------------------------------------------------------------

def select_serial_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[SIM] No serial ports detected — TCP only mode")
        return None
    if len(ports) == 1:
        print(f"[SIM] Serial port: {ports[0].device}")
        return ports[0].device
    print("\n[SIM] Available serial ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    print(f"  [{len(ports)}] None (TCP only)")
    choice = int(input("Select port number: "))
    if choice == len(ports):
        return None
    return ports[choice].device


def wait_for_connection(serial_port):
    tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    tcp_server.bind((sim_cfg["socket_host"], TCP_PORT))
    tcp_server.listen(1)
    tcp_server.setblocking(False)

    ser = None
    if serial_port:
        ser = serial.Serial(serial_port, BAUD_RATE, timeout=0)

    print(f"\n[SIM] Waiting for connection...")
    print(f"[SIM]   Serial : {serial_port or 'not available'}")
    print(f"[SIM]   TCP    : {sim_cfg['socket_host']}:{TCP_PORT}")
    print("[SIM] First to connect wins.\n")

    while True:
        try:
            conn, addr = tcp_server.accept()
            tcp_server.close()
            if ser:
                ser.close()
            print(f"[SIM] TCP connection from {addr} — SIL mode")
            return TcpTransport(conn)
        except BlockingIOError:
            pass

        if ser:
            try:
                if ser.in_waiting:
                    tcp_server.close()
                    ser.close()
                    print(f"[SIM] Serial connection on {serial_port} — HIL mode")
                    return SerialTransport(serial_port, BAUD_RATE)
            except serial.SerialException:
                pass

        time.sleep(0.05)


# ---------------------------------------------------------------
# Session logging
# ---------------------------------------------------------------

def log_session(session):
    sessions = []
    if os.path.exists(LOG_FILE):
        with open(LOG_FILE, "r") as f:
            try:
                sessions = json.load(f)
            except json.JSONDecodeError:
                sessions = []
    sessions.append(session)
    with open(LOG_FILE, "w") as f:
        json.dump(sessions, f, indent=2)


# ---------------------------------------------------------------
# Session runner
# ---------------------------------------------------------------

def run_session(transport, flight, env):
    pyro2_state   = val_cfg["pyro2"]
    true_pyro2_alt = rocket_cfg["parachute"]["trigger"]

    apogee_events = []
    pyro1_events  = []
    pyro2_events  = []

    session = {
        "timestamp":    datetime.utcnow().isoformat(),
        "mode":         transport.mode,
        "packets_sent": 0,
        "events":       [],
        "completed":    False,
    }

    t = 0.0
    print("[SIM] Streaming flight data...\n")

    while t <= flight.t_final:
        alt = flight.z(t) - env.elevation
        if alt < 0:
            break

        pressure  = flight.pressure(t) + random.gauss(0, PRESSURE_NOISE_STD)
        alt_noisy = alt                + random.gauss(0, BARO_NOISE_STD)
        ax = flight.ax(t) + random.gauss(0, ACCEL_NOISE_STD)
        ay = flight.ay(t) + random.gauss(0, ACCEL_NOISE_STD)
        az = flight.az(t) + random.gauss(0, ACCEL_NOISE_STD)
        wx = flight.w1(t) + random.gauss(0, GYRO_NOISE_STD)
        wy = flight.w2(t) + random.gauss(0, GYRO_NOISE_STD)
        wz = flight.w3(t) + random.gauss(0, GYRO_NOISE_STD)
        lat = env_cfg["latitude"]  + random.gauss(0, GPS_NOISE_STD_DEG)
        lon = env_cfg["longitude"] + random.gauss(0, GPS_NOISE_STD_DEG)

        packet = (
            f"{pressure:.2f},{ax:.4f},{ay:.4f},{az:.4f},"
            f"{wx:.4f},{wy:.4f},{wz:.4f},"
            f"{lat:.6f},{lon:.6f},{alt_noisy:.2f}\n"
        ).encode()

        try:
            transport.write(packet)
            session["packets_sent"] += 1
        except Exception as e:
            print(f"[SIM] Connection lost: {e}")
            break

        line = transport.readline()
        if line:
            parts = [p.strip() for p in line.split(",")]
            if parts[0] == "EVENT" and len(parts) >= 3:
                try:
                    event_alt = float(parts[2])
                    event = {"type": parts[1], "sim_time": round(t, 3), "alt": event_alt}
                    session["events"].append(event)
                    if parts[1] == "APOGEE":
                        apogee_events.append((t, event_alt))
                        print(f"[SIM] APOGEE at T+{t:.2f}s, alt={event_alt:.2f}m")
                    elif parts[1] == "PYRO1":
                        pyro1_events.append((t, event_alt))
                        print(f"[SIM] PYRO1 FIRE at T+{t:.2f}s, alt={event_alt:.2f}m")
                    elif parts[1] == "PYRO2":
                        pyro2_events.append((t, event_alt))
                        print(f"[SIM] PYRO2 FIRE at T+{t:.2f}s, alt={event_alt:.2f}m")
                except ValueError:
                    pass

        if REALTIME:
            time.sleep(DT)
        t += DT

    # --- Resolve detected values from event lists ---------------------------
    detected_apogee_time = apogee_events[0][0] if apogee_events else None
    detected_apogee_alt  = apogee_events[0][1] if apogee_events else None
    detected_pyro1_time  = pyro1_events[0][0]  if pyro1_events  else None
    detected_pyro1_alt   = pyro1_events[0][1]  if pyro1_events  else None
    detected_pyro2_time  = pyro2_events[0][0]  if pyro2_events  else None
    detected_pyro2_alt   = pyro2_events[0][1]  if pyro2_events  else None

    true_apogee_time = flight.apogee_time
    true_apogee_alt  = flight.apogee - env.elevation

    # --- Test report --------------------------------------------------------
    print("\n=== TEST REPORT ===")

    if detected_apogee_time is None:
        print("Apogee detection:   FAIL (never detected)")
        session["apogee_result"] = "FAIL"
    else:
        err    = abs(detected_apogee_time - true_apogee_time)
        status = "PASS" if err < val_cfg["apogee_time_tolerance_s"] else "FAIL"
        print(f"Apogee detection:   {status}")
        print(f"  True:             {true_apogee_alt:.1f}m at T+{true_apogee_time:.2f}s")
        print(f"  Detected:         {detected_apogee_alt:.1f}m at T+{detected_apogee_time:.2f}s")
        print(f"  Time error:       {err:.2f}s")
        session["apogee_result"] = status

    print("\nPyro channel 1:")
    if detected_pyro1_time is None:
        print("  FAIL (never fired)")
        session["pyro1_result"] = "FAIL"
    else:
        err    = abs(detected_pyro1_time - true_apogee_time)
        status = "PASS" if err < val_cfg["pyro1_time_tolerance_s"] else "FAIL"
        print(f"  {status} — fired at T+{detected_pyro1_time:.2f}s, alt={detected_pyro1_alt:.1f}m")
        print(f"  Time error: {err:.2f}s")
        session["pyro1_result"] = status

    if pyro2_state:
        print("\nPyro channel 2:")
        if detected_pyro2_time is None:
            print("  FAIL (never fired)")
            session["pyro2_result"] = "FAIL"
        else:
            err    = abs(detected_pyro2_alt - true_pyro2_alt)
            status = "PASS" if err < val_cfg["pyro2_alt_tolerance_m"] else "FAIL"
            print(f"  {status} — fired at T+{detected_pyro2_time:.2f}s, alt={detected_pyro2_alt:.1f}m")
            print(f"  Expected alt:   {true_pyro2_alt:.2f}m")
            print(f"  Distance error: {err:.2f}m")
            session["pyro2_result"] = status

    # --- Sequence validation ------------------------------------------------
    print("\n=== SEQUENCE VALIDATION ===")

    for name, events in [("PYRO1", pyro1_events), ("PYRO2", pyro2_events)]:
        if not pyro2_state and name == "PYRO2":
            continue
        if len(events) > 1:
            times = ", ".join(f"T+{e[0]:.2f}s" for e in events)
            print(f"{name} single-fire check: FAIL — fired {len(events)} times at: {times}")
        elif len(events) == 1:
            print(f"{name} single-fire check: PASS — fired once")
        else:
            print(f"{name} single-fire check: FAIL — never fired")

    if pyro2_state:
        if detected_pyro1_time is not None and detected_pyro2_time is not None:
            order_ok = detected_pyro2_time > detected_pyro1_time
            print(f"PYRO1 -> PYRO2 order:   {'PASS' if order_ok else 'FAIL'} "
                  f"(PYRO1 at T+{detected_pyro1_time:.2f}s, "
                  f"PYRO2 at T+{detected_pyro2_time:.2f}s)")
            gap    = detected_pyro2_time - detected_pyro1_time
            sep_ok = gap >= MIN_PYRO_SEPARATION_S
            print(f"PYRO1/PYRO2 separation: {'PASS' if sep_ok else 'FAIL'} "
                  f"(gap={gap:.2f}s, min={MIN_PYRO_SEPARATION_S}s)")
        else:
            print("PYRO1 -> PYRO2 order:   SKIPPED (one or both channels never fired)")
            print("PYRO1/PYRO2 separation: SKIPPED (one or both channels never fired)")

    session["completed"] = True
    return session

def UploadSession(session):
    if not cfg.get("telemetry", {}).get("enabled", True):
        return
    
    url = cfg.get("telemetry", {}).get("server")
    if not url:
        return

    payload = {
        "timestamp":    session.get("timestamp"),
        "mode":         session.get("mode"),
        "duration_s":   session.get("duration_s"),
        "packets_sent": session.get("packets_sent"),
        "apogee_result": session.get("apogee_result"),
        "pyro1_result":  session.get("pyro1_result"),
        "pyro2_result":  session.get("pyro2_result", "N/A"),
        "completed":    session.get("completed"),
    }

    try:
        requests.post(url, json=payload, timeout=5)
        print("[SIM] Usage data sent (opt out in config.json)")
    except Exception:
        pass


def main():
    flight, env = build_flight()
    serial_port = select_serial_port()
    transport   = wait_for_connection(serial_port)

    try:
        session = run_session(transport, flight, env)
    finally:
        transport.close()

    log_session(session)
    print(f"\n[SIM] Session logged to {LOG_FILE}")
    session_start = time.time()
    session["duration_s"] = round(time.time() - session_start, 1)
    UploadSession(session)


if __name__ == "__main__":
    main()
