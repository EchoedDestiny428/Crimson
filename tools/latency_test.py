"""Camera latency and drivetrain limits from one latency-test log (src/autons/latency_test.cpp).

    python tools/latency_test.py                      # newest logs/latency_*.csv
    python tools/latency_test.py logs/latency_004_live.csv

Get the recording with tools/latency-capture.ps1 (live, while the test runs), or afterwards from the SD card with
tools/pull-logs.ps1 and UP+DOWN on the controller (USB cable or the controller's radio; the card stays in).

Latency: while the robot wiggles and drives, the tag moves across the picture. The camera reports it a little late, so
the picture lines up best with odometry from slightly earlier. This tries every delay from 0 to 250 ms and keeps the one
where the camera and odometry agree best. That number goes into kCameraLatencyMs (include/config/camera.hpp).

Limits: speeds and accelerations come from odometry (tracking wheels and IMU) during the full-power bursts, with the
drive motors' own encoders alongside so wheel slip shows up.
"""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

import numpy as np

# include/config/camera.hpp
FX, FY, CX, CY, PITCH_DEG = 233.0, 219.0, 170.0, 115.3, 41.2
# src/robotconfig.cpp and include/config/ports.hpp: blue motors, wheels at 360 rpm, 3.25 in omnis
MOTOR_RPM, WHEEL_RPM, WHEEL_IN = 600.0, 360.0, 3.25
LEFT_PORTS, RIGHT_PORTS = (17, 18), (19, 20)

LATENCIES_MS = np.arange(0, 251, 1)
SMOOTH_SAMPLES = 5  # moving average for speeds (about 50 ms at 10 ms rows)

PHASES = {
    1: "settle", 2: "wiggle slow", 3: "wiggle medium", 4: "wiggle fast", 5: "drive slow", 6: "drive fast",
    7: "settle", 8: "forward burst", 9: "brake", 10: "reverse burst", 11: "brake", 12: "spin right", 13: "brake",
    14: "spin left", 15: "brake", 16: "return", 17: "done", -1: "aborted",
}


def load(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="") as f:
        lines = [line for line in f if not line.startswith("#") and line.strip()]
    rows = list(csv.DictReader(lines))
    if not rows:
        sys.exit(f"{path} has no rows")

    def num(v: str) -> float:
        try:
            return float(v)
        except (TypeError, ValueError):
            return math.nan

    return {key: np.array([num(r.get(key)) for r in rows]) for key in rows[0]}


def last_run(log: dict[str, np.ndarray]) -> np.ndarray:
    """Row indices of the last test run (from its last 'settle' start to where the phase goes back to 0)."""
    if "test_phase" not in log:
        sys.exit("No test_phase column: this log is not from the latency-testing branch.")
    phase = log["test_phase"]
    starts = np.flatnonzero((phase == 1) & (np.roll(phase, 1) != 1))
    if len(starts) == 0:
        sys.exit("No test run in this log (test_phase never reached 1).")
    start = starts[-1]
    end = start
    while end + 1 < len(phase) and phase[end + 1] != 0:
        end += 1
    return np.arange(start, end + 1)


def smooth(values: np.ndarray, n: int = SMOOTH_SAMPLES) -> np.ndarray:
    if len(values) < n:
        return values
    kernel = np.ones(n) / n
    padded = np.pad(values, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def camera_reads(log, rows):
    """One sample per camera read: (read time, raw u, raw v), skipping reads with no tag."""
    t_read = log["vision_read_ms"][rows]
    u = log["vision_raw_u_px"][rows]
    v = log["vision_raw_v_px"][rows]
    phase = log["test_phase"][rows]
    seen, out = set(), []
    for i in range(len(rows)):
        if not (math.isfinite(t_read[i]) and math.isfinite(u[i]) and math.isfinite(v[i])) or t_read[i] in seen:
            continue
        seen.add(t_read[i])
        out.append((t_read[i], u[i], v[i], phase[i]))
    return np.array(out) if out else np.zeros((0, 4))


def level_ray(u, v):
    """Turns the tilted pixel into a level ray: (bearing right of the lens axis, depression below level), degrees."""
    p = math.radians(PITCH_DEG)
    right = (u - CX) / FX
    down = (v - CY) / FY
    forward = math.cos(p) - down * math.sin(p)
    drop = math.sin(p) + down * math.cos(p)
    return np.degrees(np.arctan2(right, forward)), np.degrees(np.arctan2(drop, np.hypot(forward, right)))


def fit_rms(x, y, degree=1):
    if len(x) < degree + 3 or np.ptp(x) < 1e-6:
        return math.inf
    coeffs = np.polyfit(x, y, degree)
    return float(np.sqrt(np.mean((np.polyval(coeffs, x) - y) ** 2)))


def scan(reads, t, signal, response):
    """RMS misfit for every latency: the camera value at read time against odometry `signal` that much earlier."""
    out = []
    for lat in LATENCIES_MS:
        odom = np.interp(reads[:, 0] - lat, t, signal)
        out.append(fit_rms(odom, response))
    return np.array(out)


def report_latency(log, rows, reads):
    t = log["time_ms"][rows]
    heading = log["pose_theta_deg"][rows]
    x, y = log["pose_x_in"][rows], log["pose_y_in"][rows]
    bearing, depression = level_ray(reads[:, 1], reads[:, 2])

    start_heading = math.radians(heading[0])
    along = (x - x[0]) * math.cos(start_heading) - (y - y[0]) * math.sin(start_heading)
    cot_depression = 1.0 / np.tan(np.radians(np.clip(depression, 1.0, 89.0)))

    print("Camera latency (best delay, then how much worse 25 ms either side fits)")
    results = []
    for name, phases, signal, response in (
        ("wiggle (turning)", (2, 3, 4), heading, bearing),
        ("drive (toward/away)", (5, 6), along, cot_depression),
    ):
        mask = np.isin(reads[:, 3], phases)
        if mask.sum() < 10:
            print(f"  {name:22s} not enough camera reads ({int(mask.sum())}); was the tag in view?")
            continue
        curve = scan(reads[mask], t, signal, response[mask])
        best = int(LATENCIES_MS[np.argmin(curve)])
        sides = [curve[i] for i in (best - 25, best + 25) if 0 <= i < len(curve)]
        sharp = min(sides) / curve[best] if curve[best] > 0 else math.inf
        per_phase = []
        for ph in phases:
            m = reads[:, 3] == ph
            if m.sum() >= 10:
                c = scan(reads[m], t, signal, response[m])
                per_phase.append(f"{PHASES[ph]} {int(LATENCIES_MS[np.argmin(c)])}")
        print(f"  {name:22s} {best:4d} ms   x{sharp:4.2f} worse at +-25 ms   ({int(mask.sum())} reads; "
              + ", ".join(per_phase) + ")")
        results.append((best, sharp))
    if results:
        weights = [max(s - 1.0, 0.01) for _, s in results]
        combined = round(sum(b * w for (b, _), w in zip(results, weights)) / sum(weights))
        spread = max(b for b, _ in results) - min(b for b, _ in results)
        print(f"  -> kCameraLatencyMs = {combined}" + (f"   (the two tests differ by {spread} ms)" if len(results) > 1
                                                       else ""))
        print("     A sharpness under about x1.10 means the curve is flat there: the run moved too little to tell.")


def motor_wheel_speed(log, rows, ports, side):
    cols = [f"{side}_motor_rpm"] if f"{side}_motor_rpm" in log else [
        f"port_{p}_velocity" for p in ports if f"port_{p}_velocity" in log]
    if not cols:
        return None
    rpm = np.nanmean(np.vstack([log[c][rows] for c in cols]), axis=0)
    return rpm * (WHEEL_RPM / MOTOR_RPM) * math.pi * WHEEL_IN / 60.0


def report_limits(log, rows):
    t = log["time_ms"][rows] / 1000.0
    phase = log["test_phase"][rows]
    x, y, heading = log["pose_x_in"][rows], log["pose_y_in"][rows], log["pose_theta_deg"][rows]
    burst = np.flatnonzero(phase == 8)
    if len(burst) == 0:
        print("\nNo full-power bursts in this run (aborted before phase 8?).")
        return
    h0 = math.radians(heading[burst[0]])
    along = (x - x[burst[0]]) * math.cos(h0) - (y - y[burst[0]]) * math.sin(h0)
    speed = smooth(np.gradient(along, t))
    accel = smooth(np.gradient(speed, t))
    turn = smooth(np.gradient(heading, t))
    turn_accel = smooth(np.gradient(turn, t))

    def in_phase(values, *phases, pick=np.max):
        m = np.isin(phase, phases)
        return float(pick(values[m])) if m.any() else math.nan

    def time_to(values, ph, frac):
        idx = np.flatnonzero(phase == ph)
        if len(idx) == 0:
            return math.nan
        peak = np.max(np.abs(values[idx]))
        hit = idx[np.abs(values[idx]) >= frac * peak]
        return (t[hit[0]] - t[idx[0]]) * 1000.0 if len(hit) else math.nan

    left = motor_wheel_speed(log, rows, LEFT_PORTS, "left")
    right = motor_wheel_speed(log, rows, RIGHT_PORTS, "right")

    print("\nDriving (odometry; full power, then BRAKE; accelerations are peaks averaged over about 50 ms)")
    print(f"  top speed forward   {in_phase(speed, 8, 9):6.1f} in/s   reverse {-in_phase(speed, 10, 11, pick=np.min):6.1f}"
          f" in/s   (theory {WHEEL_RPM * math.pi * WHEEL_IN / 60:.1f})")
    print(f"  acceleration        {in_phase(accel, 8):6.1f} in/s^2  (reverse {-in_phase(accel, 10, pick=np.min):.1f});"
          f" 90% of top speed after {time_to(speed, 8, 0.9):.0f} ms")
    print(f"  braking             {-in_phase(accel, 9, pick=np.min):6.1f} in/s^2  "
          f"(reverse {in_phase(accel, 11):.1f})")
    if left is not None and right is not None:
        wheels = (left + right) / 2.0
        print(f"  drive motors say    {in_phase(wheels, 8, 9):6.1f} in/s forward; a lot more than odometry means the "
              "wheels slip")
    print("\nTurning (odometry heading)")
    print(f"  top turn rate       {in_phase(turn, 12, 13):6.0f} deg/s right   {-in_phase(turn, 14, 15, pick=np.min):6.0f}"
          " deg/s left")
    print(f"  turn acceleration   {in_phase(turn_accel, 12):6.0f} deg/s^2 right  "
          f"{-in_phase(turn_accel, 14, pick=np.min):6.0f} deg/s^2 left;"
          f" 90% of top rate after {time_to(turn, 12, 0.9):.0f} ms")
    print(f"  turn braking        {-in_phase(turn_accel, 13, pick=np.min):6.0f} deg/s^2  "
          f"(left {in_phase(turn_accel, 15):.0f})")


def main():
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        logs = sorted(Path("logs").glob("latency_*.csv"), key=lambda p: p.stat().st_mtime)
        if not logs:
            sys.exit("No logs/latency_*.csv. Run tools/latency-capture.ps1 during the test, or pass a path.")
        path = logs[-1]
    log = load(path)
    rows = last_run(log)
    phases_seen = sorted({int(p) for p in log["test_phase"][rows]})
    print(f"{path}: run of {len(rows)} rows, phases " + ", ".join(PHASES.get(p, str(p)) for p in phases_seen))
    if -1 in phases_seen:
        print("  (the run was aborted; results only cover the phases that ran)")
    print()
    reads = camera_reads(log, rows)
    report_latency(log, rows, reads)
    report_limits(log, rows)


if __name__ == "__main__":
    main()
