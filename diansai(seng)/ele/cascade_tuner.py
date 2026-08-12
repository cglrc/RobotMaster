#!/usr/bin/env python3
"""
Cascade PID Auto-Tuner for 3-loop Beam-Ball Balancer (IDENTIFY mode)
====================================================================
Reads VOFA JustFloat frames from the firmware running with
IDENTIFY_TEST_ENABLE=1.  The firmware steps the INNER tilt loop through
0 -> +/-3 -> 0 -> +/-6 -> 0 -> +/-9 -> 0 deg while the ball rolls free.

Plant model:  x'' = a * theta   (double integrator)
  a  = ball acceleration per degree of tilt  [px/s^2/deg]

For each constant-tilt segment the ball velocity ramps linearly:
  v(t) = v0 + a * theta * t
So fitting the velocity slope of each segment gives  a_i = slope / theta.

Then the 3-loop gains are designed from the identified `a`:

  Velocity loop (PI): theta = Kp_vel*(v_ref-v) + Ki_vel*int(v_ref-v)
      closed loop  s^2 + a*Kp_vel*s + a*Ki_vel = 0
      ->  Ki_vel = wn^2 / a ,  Kp_vel = 2*zeta*wn / a

  Outer position loop (P): v_ref = Kp_pos*(target - x)
      ->  Kp_pos = wc_pos  (choose wc_pos ~ wn/4)

Usage:
  python cascade_tuner.py --port COM17            # live fit + design
  python cascade_tuner.py --port COM17 --wn 4.0 --zeta 0.8
  python cascade_tuner.py --port COM17 --cycles 2 # stop after 2 full cycles
"""

import argparse
import sys
import threading
import time
import struct
from collections import deque
from dataclasses import dataclass, field

import serial

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

JUSTFLOAT_TAIL = 0x7F800000
FRAME_SIZE = 48
NUM_CHANNELS = 11

# Firmware segment table (must match s_identify_tilt_seq in main.c)
IDENTIFY_SEQ = [0, 3, 0, -3, 0, 6, 0, -6, 0, 9, 0, -9, 0]

# Skip the first part of each segment (motor settling + velocity-filter transient)
SEG_SKIP_S = 0.20
# Minimum samples required in a segment to fit a slope
SEG_MIN_SAMPLES = 15
# Minimum tilt to accept a segment (deg)
MIN_TILT_DEG = 1.0
# Reject segments where the ball sits near the end-stop (rolls nowhere)
WALL_POS_PX = 200.0


@dataclass
class Frame:
    det: float
    pos: float
    vel: float
    tilt: float
    offset: float
    seg: float
    ts: float


class JustFloatParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while len(self.buf) >= FRAME_SIZE:
            tail = struct.unpack_from('<I', self.buf, FRAME_SIZE - 4)[0]
            if tail == JUSTFLOAT_TAIL:
                f = struct.unpack_from('<%df' % (FRAME_SIZE // 4), self.buf, 0)
                out.append(Frame(det=f[0], pos=f[1], vel=f[2], tilt=f[3],
                                 offset=f[8], seg=f[10], ts=time.time()))
                del self.buf[:FRAME_SIZE]
            else:
                del self.buf[0]
        return out


class SerialReader:
    def __init__(self, port: str, baud: int = 115200):
        self.parser = JustFloatParser()
        self.lock = threading.Lock()
        self.queue = deque()
        self.running = False
        self.ser = serial.Serial(port, baud, timeout=0.1)

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if getattr(self, 'thread', None):
            self.thread.join(timeout=1.0)
        self.ser.close()

    def _run(self):
        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if data:
                    frames = self.parser.feed(data)
                    if frames:
                        with self.lock:
                            self.queue.extend(frames)
            except Exception as e:
                sys.stderr.write(f"serial error: {e}\n")
                time.sleep(0.1)

    def pop(self):
        with self.lock:
            ret = list(self.queue)
            self.queue.clear()
        return ret


def fit_slope(ts, vel):
    """Least-squares slope of vel vs time."""
    ts = np.asarray(ts, dtype=float)
    vel = np.asarray(vel, dtype=float)
    t0 = ts[0]
    t = ts - t0
    A = np.vstack([t, np.ones_like(t)]).T
    coef, *_ = np.linalg.lstsq(A, vel, rcond=None)
    slope, intercept = coef
    pred = A @ coef
    ss_res = float(np.sum((vel - pred) ** 2))
    ss_tot = float(np.sum((vel - np.mean(vel)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return slope, intercept, r2, len(t)
def design_gains(a, wn=4.0, zeta=0.8, wc_pos=None):
    """Design the 3 cascade gains from plant gain a [px/s^2/deg]."""
    if wc_pos is None:
        wc_pos = wn / 4.0
    kp_vel = 2.0 * zeta * wn / a
    ki_vel = wn * wn / a
    kp_pos = wc_pos
    return {
        'a': a,
        'wn': wn,
        'zeta': zeta,
        'wc_pos': wc_pos,
        'kp_vel': kp_vel,
        'ki_vel': ki_vel,
        'kp_pos': kp_pos,
    }


def simulate_cascade(g, step_px=80.0, duration=8.0, dt=0.005,
                     vel_limit=10.0, pos_limit=300.0):
    """Simulate the 3-loop cascade to sanity-check the design."""
    a = g['a']
    n = int(duration / dt)
    t = np.linspace(0, duration, n)
    x = 0.0
    v = 0.0
    vel_int = 0.0
    theta = 0.0
    sp = step_px
    xs = []
    thetas = []
    for i in range(1, n):
        # position loop (P)
        err = sp - x
        v_ref = g['kp_pos'] * err
        v_ref = np.clip(v_ref, -pos_limit, pos_limit)

        # velocity loop (PI), theta in deg
        vel_err = v_ref - v
        vel_int += vel_err * dt
        vel_int = np.clip(vel_int, -1e6, 1e6)
        theta = g['kp_vel'] * vel_err + g['ki_vel'] * vel_int
        theta = np.clip(theta, -vel_limit, vel_limit)

        # plant: x'' = a * theta
        acc = a * theta
        v += acc * dt
        x += v * dt
        xs.append(x)
        thetas.append(theta)

    xs = np.array(xs)
    thetas = np.array(thetas)
    overshoot = float((np.max(xs) - sp) / sp * 100.0)
    # settling within 5% of final
    final = xs[-1]
    band = 0.05 * sp
    settle = None
    for i in range(len(xs) - 1, -1, -1):
        if abs(xs[i] - final) > band:
            settle = t[i]
            break
    if settle is None:
        settle = 0.0
    theta_peak = float(np.max(np.abs(thetas)))
    return {
        'overshoot_pct': round(overshoot, 1),
        'settle_s': round(float(settle), 2),
        'theta_peak_deg': round(theta_peak, 1),
        'final_px': round(float(final), 1),
    }


class CascadeTuner:
    def __init__(self, port, baud=115200, wn=4.0, zeta=0.8, cycles=1):
        self.reader = SerialReader(port, baud)
        self.wn = wn
        self.zeta = zeta
        self.cycles = cycles
        self.segments = {}      # seg -> list of (t, vel)
        self.seg_start = {}
        self.last_seg = None
        self.as_ = []
        self.fit_count = 0
        self.done = False

    def _finalize_segment(self, seg, tilt):
        samples = self.segments.pop(seg, [])
        if len(samples) < SEG_MIN_SAMPLES or abs(tilt) < MIN_TILT_DEG:
            return
        # 丢弃贴墙数据：段内球平均 |pos| 过大（靠挡块，滚不动）
        pos_abs_mean = float(np.mean([abs(s[2]) for s in samples]))
        if pos_abs_mean > WALL_POS_PX:
            print(f"  seg {seg:2d} tilt={tilt:+3.1f}deg  SKIP: ball near wall "
                  f"(avg|pos|={pos_abs_mean:.0f}px>={WALL_POS_PX}px)")
            return
        # 用段内实际有效倾角（含零位补偿）的均值
        tilt_eff = float(np.mean([s[3] for s in samples]))
        if abs(tilt_eff) < MIN_TILT_DEG:
            return
        ts = [s[0] for s in samples]
        vel = [s[1] for s in samples]
        slope, intercept, r2, npts = fit_slope(ts, vel)
        a_i = slope / tilt_eff
        self.as_.append(a_i)
        self.fit_count += 1
        print(f"  seg {seg:2d} tilt_eff={tilt_eff:+3.2f}deg  "
              f"slope={slope:+8.1f}px/s^2  a={a_i:8.2f}  R2={r2:.2f}  n={npts}")

    def _report(self):
        if not self.as_:
            return
        arr = np.array(self.as_)
        a = float(np.mean(arr))
        a_std = float(np.std(arr))
        print(f"\n  plant gain a = {a:.2f} +/- {a_std:.2f} px/s^2/deg "
              f"({len(arr)} segments)")
        if a_std > 0.3 * abs(a):
            print("  [!] high variance -- re-run or check segments")
        g = design_gains(a, wn=self.wn, zeta=self.zeta)
        print(f"\n  DESIGN (wn={self.wn} rad/s, zeta={self.zeta}):")
        print(f"    stab_beam_velocity_init({g['kp_vel']:.3f}f, {g['ki_vel']:.3f}f);")
        print(f"    stab_beam_outer_init({g['kp_pos']:.3f}f);")
        print(f"    stab_beam_velocity_set_limit(10.0f);")
        if HAS_NUMPY:
            sim = simulate_cascade(g)
            print(f"  SIM step 0->80px: OS={sim['overshoot_pct']}% "
                  f"settle={sim['settle_s']}s theta_peak={sim['theta_peak_deg']}deg")

    def run(self):
        self.reader.start()
        print(f"cascade_tuner -- listening on {self.reader.ser.port} "
              f"@ {self.reader.ser.baudrate} baud")
        print("Waiting for identify segments (0 -> +/-3 -> +/-6 -> +/-9 deg)...")
        print("Ctrl-C to stop.\n")
        last_frame_ts = time.time()
        # 每个完整的 13 段序列含 6 个非零倾角段
        fits_per_cycle = sum(1 for t in IDENTIFY_SEQ if abs(t) >= MIN_TILT_DEG)
        last_offset = None
        try:
            while not self.done:
                frames = self.reader.pop()
                for f in frames:
                    last_frame_ts = f.ts
                    if last_offset is None or abs(f.offset - last_offset) > 1e-4:
                        last_offset = f.offset
                        print(f"  zero offset = {f.offset:+.3f} deg  "
                              f"(tilt_eff = seq + offset)")
                    seg = int(round(f.seg))
                    if seg < 0 or seg >= len(IDENTIFY_SEQ):
                        continue
                    tilt = IDENTIFY_SEQ[seg]
                    if self.last_seg is not None and seg != self.last_seg:
                        self._finalize_segment(self.last_seg, IDENTIFY_SEQ[self.last_seg])
                    self.last_seg = seg
                    if tilt == 0.0:
                        continue
                    if f.det < 0.5:
                        continue          # 丢球：速度清零，样本不可用
                    if seg not in self.seg_start:
                        self.seg_start[seg] = f.ts
                    rel_t = f.ts - self.seg_start[seg]
                    if rel_t >= SEG_SKIP_S:
                        self.segments.setdefault(seg, []).append(
                            (rel_t, f.vel, f.pos, f.tilt))
                    if self.fit_count >= fits_per_cycle * self.cycles:
                        self._report()
                        self.done = True
                        break
                if self.done:
                    break
                if time.time() - last_frame_ts > 3.0:
                    sys.stderr.write("\n[!] no data for 3s -- check port/firmware\n")
                    time.sleep(0.5)
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            self.reader.stop()


def main():
    parser = argparse.ArgumentParser(description="Cascade PID auto-tuner")
    parser.add_argument('--port', '-p', required=True)
    parser.add_argument('--baud', '-b', type=int, default=115200)
    parser.add_argument('--wn', type=float, default=4.0,
                        help='velocity-loop natural freq (rad/s)')
    parser.add_argument('--zeta', type=float, default=0.8,
                        help='velocity-loop damping')
    parser.add_argument('--cycles', type=int, default=1,
                        help='full cycles to collect before report')
    args = parser.parse_args()

    tuner = CascadeTuner(args.port, args.baud, args.wn, args.zeta, args.cycles)
    tuner.run()


if __name__ == '__main__':
    main()
