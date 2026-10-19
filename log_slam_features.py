#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker
import csv
import sys
import os

class SlamFeatureLogger(Node):
    def __init__(self, output_csv="slam_features.csv"):
        super().__init__('slam_feature_logger')
        self.output_csv = output_csv
        self.sub = self.create_subscription(
            MarkerArray,
            '/ov_msckf/slam_features',
            self.feature_callback,
            10
        )
        self.file = open(self.output_csv, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp', 'feature_id', 'x', 'y', 'z'])
        self.file.flush()
        self.get_logger().info(f"Subscribed to /ov_msckf/slam_features. Saving to '{self.output_csv}'...")
        print(f"\n{'Timestamp':<14} | {'Feature ID':<10} | {'X (m)':<9} | {'Y (m)':<9} | {'Z (m)':<9}")
        print("-" * 65)

    def feature_callback(self, msg: MarkerArray):
        count = 0
        for marker in msg.markers:
            # Skip delete-all action and text markers (only process 3D sphere markers)
            if marker.action == Marker.DELETEALL or marker.ns != "slam_features":
                continue

            t = marker.header.stamp.sec + marker.header.stamp.nanosec * 1e-9
            fid = marker.id
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z

            # Write to CSV
            self.writer.writerow([f"{t:.4f}", fid, f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])
            # Print to terminal
            print(f"{t:<14.4f} | {fid:<10} | {x:<9.4f} | {y:<9.4f} | {z:<9.4f}")
            count += 1

        if count > 0:
            self.file.flush()

    def destroy_node(self):
        if hasattr(self, 'file') and self.file:
            self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "slam_features.csv"
    node = SlamFeatureLogger(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping logger...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
