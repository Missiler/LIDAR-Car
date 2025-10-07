#!/usr/bin/env python3
import math
from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from autocar_interface.msg import DrivingCommand
import pigpio

# GPIO pins (BCM)
PIN_ESC = 18
PIN_SERVO = 12

class ESCServoNode(Node):
    def __init__(self):
        super().__init__('esc_servo_node')

        # ===== Parameters (can also be set via ROS params) =====
        # ESC calibration
        self.declare_parameter('esc_min_us', 1000)
        self.declare_parameter('esc_max_us', 2000)
        self.declare_parameter('esc_neutral_us', 1500)
        self.declare_parameter('esc_deadband_pct', 2.0)         # % around 0 that maps to neutral
        self.declare_parameter('esc_flip', False)               # invert throttle direction
        self.declare_parameter('esc_ramp_us_per_s', 600.0)      # throttle slew rate
        self.declare_parameter('esc_brake_mode', True)          # require brake before reverse (most RC car ESCs)
        self.declare_parameter('esc_brake_time_s', 0.25)        # how long to hold brake/neutral
        self.declare_parameter('esc_reverse_threshold_pct', 8.0)# min |%| before we consider it a real direction change

        # Steering calibration
        self.declare_parameter('servo_min_us', 1100)
        self.declare_parameter('servo_center_us', 1500)
        self.declare_parameter('servo_max_us', 1900)
        self.declare_parameter('steer_min_deg', -30.0)          # command range you intend to send
        self.declare_parameter('steer_max_deg',  30.0)
        self.declare_parameter('steer_flip', False)             # invert steering direction
        self.declare_parameter('steer_trim_us', 0)              # small offset to center (e.g. +10)
        self.declare_parameter('steer_ramp_us_per_s', 1200.0)   # steering slew rate
        self.declare_parameter('steer_expo', 0.3)               # 0..0.8 (0 = linear)

        # Behavior
        self.declare_parameter('failsafe_timeout_s', 0.5)       # if no cmd, go neutral
        self.declare_parameter('output_hz', 50.0)               # pigpio update loop

        # Load params
        p = self.get_parameter
        self.esc_min     = int(p('esc_min_us').value)
        self.esc_max     = int(p('esc_max_us').value)
        self.esc_neutral = int(p('esc_neutral_us').value)
        self.esc_deadband_pct = float(p('esc_deadband_pct').value)
        self.esc_flip    = bool(p('esc_flip').value)
        self.esc_ramp    = float(p('esc_ramp_us_per_s').value)
        self.esc_brake_mode   = bool(p('esc_brake_mode').value)
        self.esc_brake_time_s = float(p('esc_brake_time_s').value)
        self.esc_reverse_threshold_pct = float(p('esc_reverse_threshold_pct').value)

        self.servo_min   = int(p('servo_min_us').value)
        self.servo_max   = int(p('servo_max_us').value)
        self.servo_center= int(p('servo_center_us').value)
        self.steer_min_d = float(p('steer_min_deg').value)
        self.steer_max_d = float(p('steer_max_deg').value)
        self.steer_flip  = bool(p('steer_flip').value)
        self.steer_trim  = int(p('steer_trim_us').value)
        self.steer_ramp  = float(p('steer_ramp_us_per_s').value)
        self.steer_expo  = float(p('steer_expo').value)

        self.failsafe_timeout_s = float(p('failsafe_timeout_s').value)
        self.output_hz          = float(p('output_hz').value)
        self.dt                 = 1.0 / self.output_hz

        # ===== pigpio init =====
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("pigpio daemon not running. Start it with: sudo pigpiod")

        # Center/neutral on startup
        self.pi.set_servo_pulsewidth(PIN_ESC, self.esc_neutral)
        self.pi.set_servo_pulsewidth(PIN_SERVO, self.servo_center + self.steer_trim)

        # ===== State =====
        self.last_cmd_time = self.get_clock().now()
        self.target_throttle_pct = 0.0  # commanded in %
        self.target_steer_deg    = 0.0  # commanded in degrees

        self.curr_esc_us   = float(self.esc_neutral)
        self.curr_servo_us = float(self.servo_center + self.steer_trim)

        # For brake-then-reverse logic
        self._mode = 'normal'  # 'normal', 'brake_to_reverse', 'brake_to_forward'
        self._brake_end_time = None
        self._last_dir = 0  # -1, 0, +1 based on current output

        # ===== ROS I/O =====
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.subscription = self.create_subscription(
            DrivingCommand, 'topic', self.listener_callback, qos
        )
        self.timer = self.create_timer(self.dt, self.update_outputs)

        self.get_logger().info(
            f"ESC+Servo ready. ESC [{self.esc_min}-{self.esc_max}]µs, neutral={self.esc_neutral}µs; "
            f"Servo [{self.servo_min}-{self.servo_max}]µs, center={self.servo_center}µs; "
            f"50Hz loop with slew limiting."
        )

    # ---------- Helpers ----------
    @staticmethod
    def _clamp(x, a, b):
        return max(a, min(b, x))

    @staticmethod
    def _lerp(a, b, t):
        return a + (b - a) * t

    def _apply_expo(self, x, expo):
        """
        Standard RC expo-ish curve: blends linear with cubic.
        x in [-1,1]; expo in [0..~0.8]. Higher = softer around center.
        """
        x = self._clamp(x, -1.0, 1.0)
        e = self._clamp(expo, 0.0, 0.95)
        return (1 - e) * x + e * (x ** 3)

    def _percent_to_esc_us(self, pct):
        """
        Map throttle percent [-100..100] to microseconds with neutral centered.
        Respects deadband and optional inversion.
        """
        pct = self._clamp(pct, -100.0, 100.0)
        if self.esc_flip:
            pct = -pct

        # deadband around zero -> neutral
        if abs(pct) < self.esc_deadband_pct:
            return float(self.esc_neutral)

        # Symmetric span around neutral, stay within [esc_min, esc_max]
        up_span   = self.esc_max - self.esc_neutral
        down_span = self.esc_neutral - self.esc_min
        span = min(up_span, down_span)  # be safe if asymmetric
        return float(self.esc_neutral + span * (pct / 100.0))

    def _deg_to_servo_us(self, deg):
        """
        Map steering degrees to pulse, applying expo, trim, and inversion.
        deg in [steer_min_d .. steer_max_d].
        """
        deg = self._clamp(deg, self.steer_min_d, self.steer_max_d)
        if self.steer_flip:
            deg = -deg

        # Normalize to [-1,1] over declared range
        # Use the larger of |min| or |max| so 0 stays centered if ranges are asymmetric
        max_abs = max(abs(self.steer_min_d), abs(self.steer_max_d))
        x = self._clamp(deg / max_abs, -1.0, 1.0)

        # Expo for finer control around center
        x = self._apply_expo(x, self.steer_expo)

        # Interpolate between endpoints
        if x >= 0:
            us = self._lerp(self.servo_center, self.servo_max, x)
        else:
            us = self._lerp(self.servo_center, self.servo_min, -x)

        return float(us + self.steer_trim)

    # ---------- ROS subscriber ----------
    def listener_callback(self, msg: DrivingCommand):
        # Expecting msg.speedproc in [-100..100], msg.angle in degrees (roughly ±30 by default)
        spd = float(self._clamp(msg.speedproc, -100.0, 100.0))
        ang = float(self._clamp(msg.angle, self.steer_min_d, self.steer_max_d))
        self.target_throttle_pct = spd
        self.target_steer_deg = ang
        self.last_cmd_time = self.get_clock().now()

    # ---------- Output loop ----------
    def update_outputs(self):
        now = self.get_clock().now()

        # Failsafe to neutral if stale
        age = (now - self.last_cmd_time).nanoseconds / 1e9
        if age > self.failsafe_timeout_s:
            target_esc_us = float(self.esc_neutral)
            target_servo_us = float(self.servo_center + self.steer_trim)
        else:
            target_esc_us = self._percent_to_esc_us(self.target_throttle_pct)
            target_servo_us = self._deg_to_servo_us(self.target_steer_deg)

        # ----- Brake-then-reverse logic (for typical RC car ESCs) -----
        if self.esc_brake_mode:
            desired_dir = 0
            pct = self.target_throttle_pct if not self.esc_flip else -self.target_throttle_pct
            if abs(pct) >= self.esc_reverse_threshold_pct:
                desired_dir = 1 if pct > 0 else -1

            curr_dir = 0
            if self.curr_esc_us > self.esc_neutral + 5:
                curr_dir = 1
            elif self.curr_esc_us < self.esc_neutral - 5:
                curr_dir = -1

            # Initiate braking when requesting opposite direction
            if self._mode == 'normal' and desired_dir != 0 and curr_dir != 0 and desired_dir != curr_dir:
                # Hold neutral to trigger brake
                self._mode = 'brake_to_reverse' if desired_dir < 0 else 'brake_to_forward'
                self._brake_end_time = now.nanoseconds / 1e9 + self.esc_brake_time_s
                target_esc_us = float(self.esc_neutral)

            # Continue braking until timer elapses
            if self._mode.startswith('brake') and now.nanoseconds / 1e9 < self._brake_end_time:
                target_esc_us = float(self.esc_neutral)
            elif self._mode.startswith('brake') and now.nanoseconds / 1e9 >= self._brake_end_time:
                self._mode = 'normal'  # allow reverse/forward on next steps

        # ----- Slew-rate limiters -----
        def approach(curr, target, max_step):
            if target > curr:
                return min(target, curr + max_step)
            else:
                return max(target, curr - max_step)

        esc_step = self.esc_ramp * self.dt
        srv_step = self.steer_ramp * self.dt
        self.curr_esc_us   = approach(self.curr_esc_us,   target_esc_us,   esc_step)
        self.curr_servo_us = approach(self.curr_servo_us, target_servo_us, srv_step)

        # Track direction for info/debug
        if self.curr_esc_us > self.esc_neutral + 5:
            self._last_dir = 1
        elif self.curr_esc_us < self.esc_neutral - 5:
            self._last_dir = -1
        else:
            self._last_dir = 0

        # ----- Output to pigpio -----
        esc_int = int(round(self.curr_esc_us))
        srv_int = int(round(self.curr_servo_us))
        self.pi.set_servo_pulsewidth(PIN_ESC, esc_int)
        self.pi.set_servo_pulsewidth(PIN_SERVO, srv_int)

        # Periodic low-rate log (every ~0.5 s)
        if not hasattr(self, '_log_accum'):
            self._log_accum = 0.0
        self._log_accum += self.dt
        if self._log_accum >= 0.5:
            self._log_accum = 0.0
            self.get_logger().info(
                f"cmd: {self.target_throttle_pct:+5.1f}% → ESC {esc_int}µs "
                f"(mode={self._mode}) | angle: {self.target_steer_deg:+5.1f}° → SERVO {srv_int}µs"
            )

    # ---------- Shutdown ----------
    def destroy_node(self):
        self.get_logger().info("Shutting down ESC + Servo safely...")
        try:
            self.pi.set_servo_pulsewidth(PIN_ESC, self.esc_neutral)
            self.pi.set_servo_pulsewidth(PIN_SERVO, self.servo_center + self.steer_trim)
            sleep(0.5)
            self.pi.set_servo_pulsewidth(PIN_ESC, 0)
            self.pi.set_servo_pulsewidth(PIN_SERVO, 0)
            self.pi.stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESCServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
