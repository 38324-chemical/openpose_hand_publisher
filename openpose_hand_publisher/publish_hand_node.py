#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HandPublisher(Node):
    def __init__(self):
        super().__init__('hand_publisher')
        self.publisher_ = self.create_publisher(Point, 'hand_point', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz

        self.json_dir = '/home/lyx1109/openpose_json'
        self.json_files = sorted([
            f for f in os.listdir(self.json_dir)
            if f.endswith('.json')
        ])
        self.current_index = 0

        self.get_logger().info("HandPublisher node started!")

    def timer_callback(self):
        if self.current_index >= len(self.json_files):
            self.get_logger().info("No more JSON files to process.")
            return

        file_path = os.path.join(self.json_dir, self.json_files[self.current_index])
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)

            if not data['people']:
                self.get_logger().warn(f"No people found in {self.json_files[self.current_index]}")
                return

            person = data['people'][0]

            # 尝试优先用左手关键点
            left_hand = person.get('hand_left_keypoints_2d', [])
            right_hand = person.get('hand_right_keypoints_2d', [])

            if left_hand and any(left_hand):
                hand_points = left_hand
                used_hand = 'left'
            elif right_hand and any(right_hand):
                hand_points = right_hand
                used_hand = 'right'
            else:
                self.get_logger().warn(f"No valid hand keypoints in {self.json_files[self.current_index]}")
                return

            # 提取第一个关键点（0-2：x,y,置信度）
            x = float(hand_points[0])
            y = float(hand_points[1])
            z = 0.0  # OpenPose 输出是 2D 的

            point = Point()
            point.x = x
            point.y = y
            point.z = z
            self.publisher_.publish(point)

            self.get_logger().info(
                f"Published hand point from JSON #{self.current_index} ({used_hand}): x={x}, y={y}, z={z}"
            )

        except Exception as e:
            self.get_logger().error(f"Error processing JSON: {e}")

        self.current_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = HandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

