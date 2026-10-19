#!/usr/bin/env python3
"""
IMU and Camera Rig Diagnostic Tool for OpenVINS
Tests axes alignment, gravity direction, rotation directions, and data rates.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
import numpy as np
import time
import os
import sys

class VioDiagnosticNode(Node):
    def __init__(self):
        super().__init__('vio_diagnostic_node')

        self.imu_sub = self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.cam_sub = self.create_subscription(Image, '/cam0/image_raw', self.cam_cb, 10)

        self.last_imu = None
        self.last_cam_time = None
        self.imu_count = 0
        self.cam_count = 0
        self.t_start = time.time()

        self.timer = self.create_timer(0.2, self.render_dashboard)
        print("\033[2J\033[H", end="") # Clear screen

    def imu_cb(self, msg):
        self.last_imu = msg
        self.imu_count += 1

    def cam_cb(self, msg):
        self.last_cam_time = time.time()
        self.cam_count += 1

    def render_dashboard(self):
        dt = time.time() - self.t_start
        if dt < 0.1:
            return

        imu_hz = self.imu_count / dt
        cam_hz = self.cam_count / dt

        # Reset counters periodically for rolling rate
        if dt > 2.0:
            self.t_start = time.time()
            self.imu_count = 0
            self.cam_count = 0

        # Move cursor to top-left
        sys.stdout.write("\033[H")

        print("=" * 65)
        print("         VIO HARDWARE & SENSOR DIAGNOSTIC DASHBOARD        ")
        print("=" * 65)
        print(f" Camera Stream: {cam_hz:5.1f} Hz  |  IMU Stream: {imu_hz:5.1f} Hz")
        print("-" * 65)

        if self.last_imu is None:
            print(" Waiting for /imu/data_raw ...")
            return

        ax = self.last_imu.linear_acceleration.x
        ay = self.last_imu.linear_acceleration.y
        az = self.last_imu.linear_acceleration.z
        norm_a = np.sqrt(ax**2 + ay**2 + az**2)

        gx = self.last_imu.angular_velocity.x
        gy = self.last_imu.angular_velocity.y
        gz = self.last_imu.angular_velocity.z
        norm_g = np.sqrt(gx**2 + gy**2 + gz**2)

        print(f" ACCELEROMETER [m/s²]  (Target gravity magnitude: ~9.81 m/s²)")
        print(f"   Ax: {ax:7.3f}   |  Ay: {ay:7.3f}   |  Az: {az:7.3f}")
        print(f"   Total Norm ||a||: {norm_a:6.2f} m/s² ", end="")
        if 9.0 <= norm_a <= 10.6:
            print("[\033[32mOK: Senses 1G Gravity\033[0m]")
        else:
            print("[\033[31mWARNING: Scale or Bias Error\033[0m]")

        print("-" * 65)
        print(f" GYROSCOPE [rad/s]      (Target when still: < 0.05 rad/s)")
        print(f"   Gx: {gx:7.4f}   |  Gy: {gy:7.4f}   |  Gz: {gz:7.4f}")
        print(f"   Rotation Rate   : {norm_g:6.4f} rad/s ", end="")
        if norm_g < 0.08:
            print("[\033[32mSTILL / STABLE\033[0m]")
        else:
            print("[\033[33mROTATING\033[0m]")

        print("-" * 65)
        # Dominant gravity axis determination
        max_axis = np.argmax([abs(ax), abs(ay), abs(az)])
        val = [ax, ay, az][max_axis]
        axis_name = ["X", "Y", "Z"][max_axis]
        sign = "+" if val > 0 else "-"

        print(f" GRAVITY VECTOR ANALYSIS:")
        print(f"   Dominant gravity axis: {sign}{axis_name} ({val:+.2f} m/s²)")
        
        print("\n [ORIENTATION CHECK GUIDE]:")
        print(" 1. Rest rig flat on table facing forward:")
        print("    -> Note which axis is pointing UP (~ +9.8 m/s²)")
        print(" 2. Tilt lens UP towards ceiling:")
        print("    -> Which axis shifts towards +9.8 or -9.8?")
        print(" 3. Rotate rig to the RIGHT (Yaw clockwise):")
        print("    -> Which gyro axis jumps (Gx, Gy, or Gz)?")
        print("=" * 65)
        print(" Press Ctrl+C to exit diagnostic.")
        sys.stdout.flush()

def main():
    rclpy.init()
    node = VioDiagnosticNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
