#!/usr/bin/env python3
"""
Cascaded PID Self-Balancing Controller
---------------------------------------
Inner loop : Pitch angle PID   -> keeps the robot upright
Outer loop : Velocity PID      -> keeps the robot from cruising/drifting

HOW TO USE THIS FILE
=====================
All the numbers you are allowed to touch while tuning live in the
"CALIBRATION ZONE" block below, each one labeled with the STEP number
it belongs to. Change ONLY the variable(s) for the step you are
currently on, re-run, observe, then move to the next step.
Do not skip steps and do not change two steps' variables at once.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # ============================================================
        #                     CALIBRATION ZONE
        # ============================================================

        
        #   - Rights itself                -> keep False
        #   - Falls faster / runs away     -> set to True
        self.invert_polarity = False

       
        #   - Creeps forward slowly  -> increase this (+0.02, +0.04 ...)
        #   - Creeps backward slowly -> decrease this (-0.02, -0.04 ...)
        # Units: radians. Keep changes small (0.005-0.02 per try).
        self.pitch_trim = 0.00

        
        # Then turn on together starting at these values.
        self.kp_vel = 0.20   # try 0.20 once you reach 
        self.ki_vel = 0.0   # try 0.05 once you reach 

        
        self.kp_pitch = 12.0
        self.kd_pitch = 0.4
        self.ki_pitch = 0.05

        # ============================================================
        #                 END OF CALIBRATION ZONE
        # ============================================================

        # --- Fixed safety / smoothing settings (do not need tuning) ---
        self.fall_cutoff_rad = 0.6       # ~34 degrees: cuts motors if exceeded
        self.pitch_integral_clamp = 0.2
        self.vel_integral_clamp = 0.5
        self.max_lean_target_rad = 0.08  # outer loop cannot demand more than ~4.5 deg
        self.max_cmd = 1.5               # m/s equivalent speed cap sent to /cmd_vel
        self.max_cmd_step = 0.05         # slew-rate limit: max change in cmd per cycle
        self.gyro_filter_alpha = 0.3     # low-pass filter for gyro rate noise
        self.vel_filter_alpha = 0.15     # low-pass filter for velocity noise (smaller = smoother)

        # --- Internal state ---
        self.current_vel = 0.0
        self.filtered_vel = 0.0
        self.pitch_integral = 0.0
        self.vel_integral = 0.0
        self.filtered_rate = 0.0
        self.last_cmd = 0.0
        self.last_time = self.get_clock().now()
        self.debug_counter = 0  # used only to slow down the debug print below

        # --- Teleop (keyboard driving) state ---
        self.desired_vel = 0.0          # requested forward/backward speed, m/s
        self.desired_yaw_rate = 0.0     # requested turn rate, rad/s
        self.last_teleop_time = self.get_clock().now()
        self.teleop_timeout_sec = 1.0   # if no teleop message arrives for
                                         # this long, treat it as "stop
                                         # requesting movement" -- a basic
                                         # safety net in case the keyboard
                                         # terminal is closed unexpectedly

        # --- ROS interfaces ---
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.teleop_sub = self.create_subscription(
            Twist, '/teleop_cmd', self.teleop_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Balance Controller Initialized.')
        self.get_logger().info(
            f'invert_polarity={self.invert_polarity}  '
            f'pitch_trim={self.pitch_trim}  '
            f'kp_vel={self.kp_vel}  ki_vel={self.ki_vel}'
        )

    # ------------------------------------------------------------------
    def quaternion_to_pitch(self, x, y, z, w):
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            return math.copysign(math.pi / 2.0, sinp)
        return math.asin(sinp)

    def odom_callback(self, msg):
        self.current_vel = msg.twist.twist.linear.x
        
        self.filtered_vel = (self.vel_filter_alpha * self.current_vel) + \
                             ((1.0 - self.vel_filter_alpha) * self.filtered_vel)

    def teleop_callback(self, msg):
        
        self.desired_vel = max(min(msg.linear.x, 0.25), -0.25)
        self.desired_yaw_rate = max(min(msg.angular.z, 1.0), -1.0)
        self.last_teleop_time = self.get_clock().now()

    def stop_motors(self):
        self.cmd_pub.publish(Twist())
        self.pitch_integral = 0.0
        self.vel_integral = 0.0
        self.last_cmd = 0.0

    # ------------------------------------------------------------------
    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0.0 or dt > 0.5:
            
            self.pitch_integral = 0.0
            self.vel_integral = 0.0
            self.last_cmd = 0.0
            self.filtered_rate = 0.0
            self.current_vel = 0.0
            self.filtered_vel = 0.0
            self.last_time = current_time
            return

        
        teleop_age = (current_time - self.last_teleop_time).nanoseconds / 1e9
        if teleop_age > self.teleop_timeout_sec:
            self.desired_vel = 0.0
            self.desired_yaw_rate = 0.0

        # 1. Read pitch and remove the calibrated trim offset.
        #    invert_polarity is applied HERE, once, to the raw sensor
        #    reading itself -- so every calculation downstream (outer
        #    velocity loop AND inner pitch loop) sees a consistent,
        #    already-correct sense of direction. This avoids the two
        #    loops disagreeing with each other.
        q = msg.orientation
        raw_pitch = self.quaternion_to_pitch(q.x, q.y, q.z, q.w)
        if self.invert_polarity:
            raw_pitch = -raw_pitch
        pitch = raw_pitch - self.pitch_trim

        # 2. Safety cutoff — robot has fallen past recoverable angle
        if abs(pitch) > self.fall_cutoff_rad:
            self.stop_motors()
            return

        # 3. Outer loop: compare current velocity against the REQUESTED
        #    velocity (0 unless the keyboard teleop is asking for
        #    motion), and produce a small target lean angle that drives
        #    the robot toward that requested speed. Uses the FILTERED
        #    velocity, not the raw reading, so sensor jitter doesn't
        #    cause constant small corrections.
        vel_error = self.filtered_vel - self.desired_vel
        self.vel_integral += vel_error * dt
        self.vel_integral = max(min(self.vel_integral, self.vel_integral_clamp),
                                 -self.vel_integral_clamp)
        target_pitch = -((self.kp_vel * vel_error) +
                          (self.ki_vel * self.vel_integral))
        target_pitch = max(min(target_pitch, self.max_lean_target_rad),
                            -self.max_lean_target_rad)

        # 4. Inner loop: pitch PID around that target
        pitch_error = pitch - target_pitch

        gyro_rate = msg.angular_velocity.y
        if self.invert_polarity:
            gyro_rate = -gyro_rate
        self.filtered_rate = (self.gyro_filter_alpha * gyro_rate) + \
                              ((1.0 - self.gyro_filter_alpha) * self.filtered_rate)

        self.pitch_integral += pitch_error * dt
        self.pitch_integral = max(min(self.pitch_integral, self.pitch_integral_clamp),
                                   -self.pitch_integral_clamp)

        raw_cmd = (self.kp_pitch * pitch_error) + \
                  (self.kd_pitch * self.filtered_rate) + \
                  (self.ki_pitch * self.pitch_integral)

        # 5. (polarity already applied above, at the sensor reading --
        #    nothing to flip here anymore)
        cmd = raw_cmd

        # 6. Speed cap + slew-rate limit (smooths sudden jumps, protects
        #    against the gearbox/driver lag you'll hit on real hardware)
        cmd = max(min(cmd, self.max_cmd), -self.max_cmd)
        cmd_delta = max(min(cmd - self.last_cmd, self.max_cmd_step), -self.max_cmd_step)
        cmd = self.last_cmd + cmd_delta
        self.last_cmd = cmd

        self.debug_counter += 1
        if self.debug_counter % 20 == 0:
            print(
                f"pitch={math.degrees(pitch):6.1f} deg | "
                f"target={math.degrees(target_pitch):5.1f} deg | "
                f"vel={self.current_vel:5.2f} m/s | "
                f"cmd={cmd:5.2f}"
            )

        # 7. Publish. Turning (angular.z) is applied directly from the
        #    teleop request -- it doesn't affect pitch/balance at all,
        #    since turning just spins the two wheels at slightly
        #    different speeds around the same forward command.
        twist = Twist()
        twist.linear.x = cmd
        twist.angular.z = self.desired_yaw_rate
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()