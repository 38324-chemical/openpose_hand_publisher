## openpose\_hand\_publisher

## 简介

这是一个基于 ROS2 的发布者节点，读取 OpenPose 生成的手部关键点 JSON 数据，实时发布手部关键点坐标到 `/hand_point` 话题。

## 目录结构

```
openpose_hand_publisher/
├── launch/
│   └── publish_hand.launch.py
├── openpose_hand_publisher/
│   └── publish_hand_node.py
├── package.xml
└── CMakeLists.txt
```

## 依赖

* ROS2 Humble
* Python 3
* geometry\_msgs
* rclpy

## 编译安装

```bash
cd ~/dev_ws
colcon build --packages-select openpose_hand_publisher
source install/setup.bash
```

## 运行发布节点

```bash
ros2 run openpose_hand_publisher publish_hand_node
```

## 消息说明

* 话题名：`/hand_point`
* 消息类型：`geometry_msgs/msg/Point`
* 内容：

  * x, y: 从 OpenPose JSON 中解析出的手部关键点坐标
  * z: 固定为 0

## 订阅示例

订阅者只需要订阅 `/hand_point`，消息类型为 `geometry_msgs/msg/Point` 即可。

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HandSubscriber(Node):
    def __init__(self):
        super().__init__('hand_subscriber')
        self.subscription = self.create_subscription(Point, 'hand_point', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received hand point: x={msg.x}, y={msg.y}, z={msg.z}")

def main(args=None):
    rclpy.init(args=args)
    node = HandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

