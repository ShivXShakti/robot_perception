#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from your_pkg_name.msg import ObjPoseArray  # replace with your package name


class ObjPoseSubscriber(Node):
    def __init__(self):
        super().__init__('obj_pose_subscriber')
        self.subscription = self.create_subscription(
            ObjPoseArray,
            'obj_pose_array',   # topic name
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg: ObjPoseArray):
        if not msg.data:
            self.get_logger().info("No objects received")
            return

        self.get_logger().info(f"Received {len(msg.data)} objects:")
        for obj in msg.data:
            pose = obj.pose_stamped.pose
            self.get_logger().info(
                f"Object: {obj.object_name}, "
                f"Position -> x:{pose.position.x:.2f}, y:{pose.position.y:.2f}, z:{pose.position.z:.2f}, "
                f"Orientation -> x:{pose.orientation.x:.2f}, y:{pose.orientation.y:.2f}, "
                f"z:{pose.orientation.z:.2f}, w:{pose.orientation.w:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObjPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
