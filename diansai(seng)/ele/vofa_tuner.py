#!/usr/bin/env python3
"""
VOFA JustFloat Monitor & PID Auto-Tuner for Gimbal Ball Balancer
================================================================
Reads VOFA JustFloat frames from a serial port, monitors the ball balance
system in real-time, captures step responses, and computes optimal PID
parameters via model-based tuning.

Usage:
  python vofa_tuner.py --port COM37                # live monitor
  python vofa_tuner.py --port COM37 --baud 115200  # custom baud
  python vofa_tuner.py --port COM37 --tune         # auto-tune on next step
  python vofa_tuner.py --port COM37 --csv log.csv  # log to CSV

VOFA JustFloat frame (48 bytes):
  11× float32 data + 1× float32 tail (0x7F800000 = +inf)

Channel map (from gimbal.cpp vofa_send):
  I0  = vision_data_1         raw camera offset (pixels)
  I1  = filteredPosition      filtered ball position (pixels)
  I2  = filteredVelocity      filtered ball velocity (pixels/s)
  I3  = targetMotorAngle      desired motor angle (deg)
  I4  = lastMotorCommand      motor speed command (deg/s)
  I5  = currentMotorAngle     actual motor angle (deg)
  I6  = angleError            target - actual motor angle (deg)
  I7  = currentMotorSpeed     actual motor speed (deg/s)
  I8  = integralOffset        PID integral term
  I9  = positionTarget        target ball position (pixels)
  I10 = state                 0=IDLE,1=SELECTING,2=HOLD_CTR,3=MOVE_+,4=MOVE_-,5=HOLD_-

Tuning method: IMC-based PID for integrating processes.
The ball-on-plate from motor-angle to ball-position approximates:
  G(s) = K / (s * (tau*s + 1))    (integrator + lag)
PID parameters are derived from the identified K and tau.
"""

import serial
import struct
import sys
import time
import signal
import argparse
import csv
import os
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import threading

# ---------------------------------------------------------------------------
# Try numpy; degrade gracefully if missing
# ---------------------------------------------------------------------------
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# ============================================================================
# JustFloat Protocol Constants
# ============================================================================
JUSTFLOAT_TAIL = 0x7F800000   # float32 +infinity
FRAME_SIZE = 48               # 12 floats × 4 bytes
NUM_CHANNELS = 11

# State name lookup
STATE_NAMES = {
    0: "IDLE",
    1: "SELECTING",
    2: "HOLD_CENTER",
    3: "MOVE_TO_PLUS",
    4: "MOVE_TO_MINUS",
    5: "HOLD_MINUS",
}

# ============================================================================
# Data structures
# ============================================================================

@dataclass
class VOFAFrame:
    """One parsed JustFloat frame."""
    raw_offset: float       # I0
    filtered_pos: float     # I1
    filtered_vel: float     # I2
    target_angle: float     # I3
    motor_cmd: float        # I4
    current_angle: float    # I5
    angle_error: float      # I6
    current_speed: float    # I7
    integral: float         # I8
    pos_target: float       # I9
    state: float            # I10
    timestamp: float = 0.0

    def state_name(self) -> str:
        return STATE_NAMES.get(int(self.state), f"UNK({int(self.state)})")


@dataclass
class StepResponse:
    """Captured step response from setpoint change."""
    setpoint_from: float
    setpoint_to: float
    start_time: float
    samples: List[Tuple[float, float, float]] = field(default_factory=list)
    # each sample: (t_relative, position, velocity)

    @property
    def time(self) -> list:
        return [s[0] for s in self.samples]

    @property
    def position(self) -> list:
        return [s[1] for s in self.samples]

    @property
    def velocity(self) -> list:
        return [s[2] for s in self.samples]

    @property
    def duration(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1][0]

    @property
    def overshoot_pct(self) -> float:
        """Overshoot percentage."""
        if not self.samples:
            return 0.0
        step = self.setpoint_to - self.setpoint_from
        if abs(step) < 0.01:
            return 0.0
        peak = max(self.position) if step > 0 else min(self.position)
        peak_err = peak - self.setpoint_to
        return 100.0 * peak_err / abs(step)

    @property
    def steady_state_error(self) -> float:
        """Error at end of capture."""
        if not self.samples:
            return 0.0
        tail_n = min(20, len(self.samples))
        avg_tail = sum(self.position[-tail_n:]) / tail_n
        return avg_tail - self.setpoint_to

    def summary(self) -> str:
        return (
            f"Step: {self.setpoint_from:.0f} -> {self.setpoint_to:.0f} px, "
            f"dur={self.duration:.2f}s, "
            f"overshoot={self.overshoot_pct:.1f}%, "
            f"ss_err={self.steady_state_error:.1f}px"
        )


# ============================================================================
# JustFloat Parser
# ============================================================================

class JustFloatParser:
    """Stateful sync-and-parse for JustFloat binary stream."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes) -> List[VOFAFrame]:
        self.buf.extend(data)
        frames: List[VOFAFrame] = []

        while len(self.buf) >= FRAME_SIZE:
            tail = struct.unpack_from('<I', self.buf, FRAME_SIZE - 4)[0]
            if tail == JUSTFLOAT_TAIL:
                f = struct.unpack_from(f'<{FRAME_SIZE // 4}f', self.buf, 0)
                frames.append(VOFAFrame(
                    raw_offset=f[0], filtered_pos=f[1], filtered_vel=f[2],
                    target_angle=f[3], motor_cmd=f[4], current_angle=f[5],
                    angle_error=f[6], current_speed=f[7], integral=f[8],
                    pos_target=f[9], state=f[10],
                    timestamp=time.time(),
                ))
                del self.buf[:FRAME_SIZE]
            else:
                del self.buf[0]  # byte-level resync

        return frames


# ============================================================================
# Step Detector
# ============================================================================

class StepDetector:
    """Watches pos_target; captures response when a step change is seen.

    Only triggers when state transitions from IDLE/SELECTING into an active
    mode (HOLD_CENTER or waypoint), ensuring we capture the real system response
    rather than noise.
    """

    def __init__(self, capture_s: float = 8.0, min_step_px: float = 10.0):
        self.capture_s = capture_s
        self.min_step_px = min_step_px
        self._last_target: Optional[float] = None
        self._capturing: Optional[StepResponse] = None
        self._done: List[StepResponse] = []

    @property
    def is_capturing(self) -> bool:
        return self._capturing is not None

    def feed(self, frame: VOFAFrame) -> Optional[StepResponse]:
        """Feed a frame.  Returns a StepResponse when capture completes."""
        released: Optional[StepResponse] = None

        # --- if a capture is in flight, record sample ---
        if self._capturing is not None:
            t = frame.timestamp - self._capturing.start_time
            if t <= self.capture_s:
                self._capturing.samples.append(
                    (t, frame.filtered_pos, frame.filtered_vel))
            if t >= self.capture_s:
                self._done.append(self._capturing)
                released = self._capturing
                self._capturing = None

        # --- detect step change on target ---
        if self._last_target is not None and self._capturing is None:
            delta = frame.pos_target - self._last_target
            if abs(delta) >= self.min_step_px:
                print(f"\n>>> Step detected: "
                      f"{self._last_target:.0f} -> {frame.pos_target:.0f} px  "
                      f"(state={frame.state_name()})")
                self._capturing = StepResponse(
                    setpoint_from=self._last_target,
                    setpoint_to=frame.pos_target,
                    start_time=frame.timestamp,
                )

        self._last_target = frame.pos_target
        return released

    def pop_completed(self) -> List[StepResponse]:
        ret = list(self._done)
        self._done.clear()
        return ret


# ============================================================================
# System Identification  (requires numpy)
# ============================================================================

def identify_integrator_lag(response: StepResponse) -> Optional[dict]:
    """Fit the step response to  G(s) = K / (s * (tau*s + 1)).

    Returns dict with keys: K, tau, fit_quality, or None on failure.
    """
    if not HAS_NUMPY:
        return None

    step = response.setpoint_to - response.setpoint_from
    if abs(step) < 0.1:
        return None

    t = np.array(response.time)
    y = (np.array(response.position) - response.setpoint_from) / step
    # y(t) should ramp toward 1 for a double-integrator-like system, or
    # settle at 1 for an integrator+lag system.
    # We fit:  y(t) = t - tau*(1-exp(-t/tau))  after scaling by K.

    # Simpler approach: use velocity profile.
    # G(s)=K/(s*(tau*s+1)).  Step response: dy/dt = K*(1 - exp(-t/tau))
    if len(t) < 10:
        return None

    vel = np.array(response.velocity) / step if abs(step) > 0.01 else np.array(response.velocity)
    # vel(t) → K as t→∞  for a stable integrating process

    # Estimate K from peak or tail velocity
    tail_n = max(5, len(vel) // 5)
    K_est = float(np.mean(np.abs(vel[-tail_n:]))) if tail_n > 0 else float(np.max(np.abs(vel)))
    if K_est < 1e-6:
        return None

    # Estimate tau from rise time of velocity to 63% of K
    vel_abs = np.abs(vel)
    target = 0.632 * K_est
    tau_est = 0.5  # default
    idx = np.argmax(vel_abs >= target)
    if idx > 0 and idx < len(t):
        tau_est = float(t[idx])

    # Refine with a rough least-squares on velocity
    # v(t) ≈ K * (1 - exp(-t/tau))
    try:
        from math import exp
        best_tau = tau_est
        best_err = 1e30
        for tau_candidate in np.linspace(max(0.01, tau_est * 0.3), tau_est * 3.0, 40):
            v_pred = K_est * (1.0 - np.exp(-t / tau_candidate))
            err = float(np.sum((vel_abs - v_pred) ** 2))
            if err < best_err:
                best_err = err
                best_tau = float(tau_candidate)
        tau_est = best_tau
    except Exception:
        pass

    # Quality metric: R² of velocity fit
    v_pred = K_est * (1.0 - np.exp(-t / tau_est))
    ss_res = float(np.sum((vel_abs - v_pred) ** 2))
    ss_tot = float(np.sum((vel_abs - np.mean(vel_abs)) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0

    return {
        'K': round(K_est, 4),
        'tau': round(tau_est, 4),
        'r2': round(r2, 3),
        'model': f'G(s) = {K_est:.4f} / (s * ({tau_est:.4f}s + 1))',
    }


# ============================================================================
# PID Tuning — IMC method for integrating processes
# ============================================================================

def tune_imc_integrating(model: dict, lambda_factor: float = 1.0) -> dict:
    """IMC-based PID tuning for G(s) = K / (s * (tau*s + 1)).

    Parameters
    ----------
    model : dict with keys K, tau
    lambda_factor : desired closed-loop speed.
                    0.5 = aggressive, 1.0 = nominal, 2.0 = conservative.

    Returns
    -------
    dict with keys: Kp, Ki, Kd, maxTargetOffset, maxMotorSpeed
    """
    K = model['K']
    tau = model['tau']

    # IMC tuning for integrating+lag process
    # lambda = closed-loop time constant
    lam = lambda_factor * tau * 2.0
    lam = max(lam, 0.02)

    Kp = (2.0 * lam + tau) / (K * lam * lam)
    Ki = 1.0 / (K * lam * lam)
    Kd = 0.0  # D is embedded in velocity feedback in our architecture

    # Our controller uses velocityToAngleGain as the damping term,
    # which acts like derivative on position.  IMC equivalent:
    #   velocityToAngleGain ≈ tau / (K * lam)
    Kd_velocity = tau / (K * lam) if K > 1e-9 else 0.0

    # Heuristic limits for this platform (pixels ↔ degrees)
    Kp = round(Kp, 4)
    Ki = round(Ki, 4)
    Kd_velocity = round(Kd_velocity, 4)

    # Suggest maxTargetOffset based on expected max error × Kp
    max_offset = round(min(30.0, max(8.0, abs(Kp) * 80.0)), 1)
    max_speed = round(min(300.0, max(80.0, abs(Kp) * 800.0)), 1)

    return {
        'positionToAngleGain': Kp,
        'positionIntegralGain': Ki,
        'velocityToAngleGain': Kd_velocity,
        'maxTargetOffset': max_offset,
        'maxMotorSpeed': max_speed,
        'lambda': round(lam, 3),
    }


def simulate_step(model: dict, pid: dict, step: float,
                  duration: float = 5.0, dt: float = 0.005) -> dict:
    """Simulate closed-loop step response with the tuned PID."""
    if not HAS_NUMPY:
        return {}
    K, tau = model['K'], model['tau']
    Kp, Ki, Kd = pid['positionToAngleGain'], pid['positionIntegralGain'], pid['velocityToAngleGain']
    max_angle = pid['maxTargetOffset']
    max_speed = pid['maxMotorSpeed']

    n = int(duration / dt)
    t = np.linspace(0, duration, n)
    y = np.zeros(n)       # ball position (integrator output)
    v = np.zeros(n)       # ball velocity
    u = np.zeros(n)       # motor angle command
    u_actual = np.zeros(n)  # motor angle (rate-limited)
    integral = 0.0
    sp = step
    ref = np.zeros(n)
    ref[:] = sp

    for i in range(1, n):
        err = sp - y[i - 1]
        integral += err * dt

        # PID
        u_raw = Kp * err + Ki * integral - Kd * v[i - 1]
        u[i] = np.clip(u_raw, -max_angle, max_angle)

        # Motor rate limit
        max_step = max_speed * dt
        u_actual[i] = u_actual[i - 1] + np.clip(u[i] - u_actual[i - 1],
                                                 -max_step, max_step)

        # Plant:  dv/dt = K * u_actual - (1/tau) * v
        #         dy/dt = v
        dv = K * u_actual[i] - (1.0 / tau) * v[i - 1]
        v[i] = v[i - 1] + dv * dt
        y[i] = y[i - 1] + v[i - 1] * dt

    # Metrics
    overshoot = float((np.max(y) - step) / abs(step) * 100) if abs(step) > 0.01 else 0
    # Settling time (within 5% of step)
    settled_idx = n - 1
    for i in range(n):
        if np.all(np.abs(y[i:] - step) <= 0.05 * abs(step)):
            settled_idx = i
            break
    settle_time = float(t[settled_idx])
    rise_time = float(t[np.argmax(y >= 0.9 * step)]) if step > 0 else float(t[np.argmax(y <= 0.9 * step)])

    return {
        'overshoot_pct': round(overshoot, 1),
        'settle_time_5pct': round(settle_time, 2),
        'rise_time_90pct': round(rise_time, 2),
        'ss_error': round(float(y[-1] - step), 3),
    }


# ============================================================================
# Live terminal display
# ============================================================================

class LiveDisplay:
    """Minimal terminal status line updated in-place."""

    def __init__(self):
        self._last_line_len = 0
        self._last_printed_count = -1
        self.frame_count = 0
        self.last_frame: Optional[VOFAFrame] = None

    def update(self, frame: VOFAFrame):
        self.last_frame = frame
        self.frame_count += 1

    @property
    def has_new(self) -> bool:
        return self.frame_count != self._last_printed_count

    def render_line(self) -> str:
        if self.last_frame is None:
            return "Waiting for data..."
        f = self.last_frame
        return (
            f"[{self.frame_count:6d}] "
            f"pos={f.filtered_pos:7.1f}px  "
            f"vel={f.filtered_vel:+7.1f}px/s  "
            f"tgt={f.pos_target:+6.0f}px  "
            f"motor={f.current_angle:+6.1f}deg  "
            f"spd={f.current_speed:+6.1f}deg/s  "
            f"state={f.state_name():14s}"
        )

    def print_line(self):
        if not self.last_frame or not self.has_new:
            return
        line = self.render_line()
        clear = ' ' * max(0, self._last_line_len - len(line))
        sys.stdout.write(f"\r{line}{clear}")
        sys.stdout.flush()
        self._last_line_len = len(line)
        self._last_printed_count = self.frame_count


# ============================================================================
# CSV Logger
# ============================================================================

class CsvLogger:
    def __init__(self, path: str):
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f, lineterminator='\n')
        self.w.writerow([
            'timestamp', 'raw_offset', 'filtered_pos', 'filtered_vel',
            'target_angle', 'motor_cmd', 'current_angle', 'angle_error',
            'current_speed', 'integral', 'pos_target', 'state',
        ])

    def write(self, frame: VOFAFrame):
        self.w.writerow([
            f"{frame.timestamp:.3f}",
            f"{frame.raw_offset:.6f}",
            f"{frame.filtered_pos:.6f}",
            f"{frame.filtered_vel:.6f}",
            f"{frame.target_angle:.6f}",
            f"{frame.motor_cmd:.6f}",
            f"{frame.current_angle:.6f}",
            f"{frame.angle_error:.6f}",
            f"{frame.current_speed:.6f}",
            f"{frame.integral:.6f}",
            f"{frame.pos_target:.6f}",
            f"{frame.state:.0f}",
        ])

    def close(self):
        self.f.close()


# ============================================================================
# Serial Reader Thread
# ============================================================================

class SerialReader:
    def __init__(self, port: str, baud: int = 115200, retries: int = 3):
        self.parser = JustFloatParser()
        self.lock = threading.Lock()
        self.frame_queue: List[VOFAFrame] = []
        self.running = False
        self.thread: Optional[threading.Thread] = None

        last_err = None
        for attempt in range(retries):
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                return
            except serial.SerialException as e:
                last_err = e
                if attempt < retries - 1:
                    sys.stderr.write(
                        f"\rPort {port} busy (attempt {attempt+1}/{retries}), "
                        f"retrying in 1s...\n")
                    time.sleep(1.0)

        # All retries failed — give helpful message
        import subprocess
        sys.stderr.write(f"\nERROR: Cannot open {port}\n")
        sys.stderr.write(f"  -> {last_err}\n\n")
        sys.stderr.write("Troubleshooting:\n")
        sys.stderr.write("  1. Close VOFA, serial monitors, and JLinkGUIServer\n")
        sys.stderr.write("  2. Unplug/replug the USB-UART or JLink adapter\n")
        sys.stderr.write("  3. Check Device Manager for the correct COM port number\n")
        sys.stderr.write("  4. Try: python -c \"import serial; serial.Serial('COMx').close()\"\n")
        raise last_err

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
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
                            self.frame_queue.extend(frames)
            except Exception as e:
                sys.stderr.write(f"\nSerial error: {e}\n")
                time.sleep(0.1)

    def pop_frames(self) -> List[VOFAFrame]:
        with self.lock:
            ret = list(self.frame_queue)
            self.frame_queue.clear()
        return ret


# ============================================================================
# Main application
# ============================================================================

class VOFATuner:
    def __init__(self, port: str, baud: int = 115200,
                 capture_s: float = 8.0, csv_path: Optional[str] = None):
        self.reader = SerialReader(port, baud)
        self.display = LiveDisplay()
        self.detector = StepDetector(capture_s=capture_s)
        self.logger = CsvLogger(csv_path) if csv_path else None
        self.responses: List[StepResponse] = []
        self.auto_tune = False
        self.running = False

    def start(self):
        self.reader.start()
        self.running = True

    def stop(self):
        self.running = False
        self.reader.stop()
        if self.logger:
            self.logger.close()

    def run_forever(self):
        """Blocking main loop; press Ctrl-C to stop."""
        self.start()
        print(f"VOFA Tuner -- listening on {self.reader.ser.port} @ {self.reader.ser.baudrate} baud")
        print("Press Ctrl-C to stop.\n")
        print("Trigger a step by pressing the button on the gimbal (single/double press).")
        print("The tuner will capture the response and suggest PID parameters.\n")

        report_count = 0
        try:
            while self.running:
                frames = self.reader.pop_frames()
                for f in frames:
                    self.display.update(f)
                    if self.logger:
                        self.logger.write(f)

                    result = self.detector.feed(f)
                    if result is not None:
                        self._on_step_captured(result)

                self.display.print_line()

                # Periodic status
                report_count += 1
                if report_count >= 40:  # ~every 2s at 50Hz
                    self._periodic_report()
                    report_count = 0

                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n\nStopped by user.")
        finally:
            self.stop()

    def _on_step_captured(self, resp: StepResponse):
        self.responses.append(resp)
        print(f"\n{'='*60}")
        print(f"Step #{len(self.responses)} captured: {resp.summary()}")
        print(f"{'='*60}")

        # System identification
        if HAS_NUMPY:
            model = identify_integrator_lag(resp)
        else:
            print("(numpy not available -- skipping system ID)")
            model = None

        if model is None:
            print("Could not fit a reliable model. Try a larger step or "
                  "ensure the ball moves freely.")
            return

        print(f"\nIdentified model:  {model['model']}")
        print(f"  K   = {model['K']:.4f}  (velocity / motor-angle)")
        print(f"  tau = {model['tau']:.4f} s  (lag time constant)")
        print(f"  R^2 = {model['r2']:.3f}  (fit quality, >0.7 is good)")

        if model['r2'] < 0.5:
            print("  [!] Low fit quality -- results may be unreliable.")

        # Tune for several lambda values
        print(f"\n{'-'*60}")
        print(f"{'Lambda':>8s}  {'Kp':>8s}  {'Ki':>8s}  {'Kd(vel)':>8s}  "
              f"{'maxAngle':>9s}  {'maxSpeed':>9s}  {'OS%':>6s}  {'Ts(s)':>6s}")
        print(f"{'-'*60}")

        for lf in [0.5, 0.8, 1.0, 1.5, 2.0]:
            pid = tune_imc_integrating(model, lambda_factor=lf)
            step_px = resp.setpoint_to - resp.setpoint_from
            sim = simulate_step(model, pid, abs(step_px))
            label = {0.5: 'fast', 0.8: 'brisk', 1.0: 'nominal',
                     1.5: 'safe', 2.0: 'gentle'}.get(lf, '')
            print(
                f"{lf:>4.1f} {label:>6s}  "
                f"{pid['positionToAngleGain']:8.4f}  "
                f"{pid['positionIntegralGain']:8.4f}  "
                f"{pid['velocityToAngleGain']:8.4f}  "
                f"{pid['maxTargetOffset']:8.1f}deg  "
                f"{pid['maxMotorSpeed']:8.1f}deg/s  "
                f"{sim.get('overshoot_pct', 0):5.1f}%  "
                f"{sim.get('settle_time_5pct', 0):5.2f}s"
            )

        # Recommended
        rec = tune_imc_integrating(model, lambda_factor=1.0)
        print(f"\n{'='*60}")
        print("RECOMMENDED PARAMETERS (paste into gimbal.cpp makeBalanceConfig):")
        print(f"{'='*60}")
        print(f"  config.positionToAngleGain   = {rec['positionToAngleGain']:.4f}f;")
        print(f"  config.velocityToAngleGain   = {rec['velocityToAngleGain']:.4f}f;")
        print(f"  config.positionIntegralGain  = {rec['positionIntegralGain']:.4f}f;")
        print(f"  config.maxTargetOffset       = {rec['maxTargetOffset']:.1f}f;")
        print(f"  config.maxMotorSpeed         = {rec['maxMotorSpeed']:.1f}f;")
        print(f"  // Keep existing: maxMotorAcceleration, filter tau values")
        print(f"  // Closed-loop lambda = {rec['lambda']:.3f}s")
        print(f"{'='*60}\n")

    def _periodic_report(self):
        if self.detector.is_capturing:
            print(f"\n  (capturing step response...)")
            sys.stderr.flush()


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="VOFA Monitor & PID Auto-Tuner for Gimbal Ball Balancer")
    parser.add_argument('--port', '-p',
                        help='Serial port (e.g. COM37)')
    parser.add_argument('--baud', '-b', type=int, default=115200,
                        help='Baud rate (default: 115200)')
    parser.add_argument('--capture', '-c', type=float, default=8.0,
                        help='Step capture duration in seconds (default: 8)')
    parser.add_argument('--csv', help='Log frames to CSV file')
    parser.add_argument('--list', action='store_true',
                        help='List available COM ports and exit')
    args = parser.parse_args()

    if args.list:
        list_ports()
        return

    if not args.port:
        parser.print_help()
        print("\nAvailable ports:")
        list_ports()
        print("\nUsage: python vofa_tuner.py --port COMx")
        return

    if args.csv:
        print(f"Logging frames to: {args.csv}")

    tuner = VOFATuner(
        port=args.port,
        baud=args.baud,
        capture_s=args.capture,
        csv_path=args.csv,
    )
    tuner.run_forever()


def list_ports():
    """List available serial ports with descriptions."""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("  No serial ports found.")
        return
    for p in ports:
        print(f"  {p.device}  -- {p.description}")


if __name__ == '__main__':
    main()
