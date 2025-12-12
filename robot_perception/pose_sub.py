#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from dualarm_custom_msgs.msg import ObjPoseArray


class ObjPoseSubscriber(Node):
    def __init__(self):
        super().__init__('obj_pose_subscriber')
        self.subscription = self.create_subscription(
            ObjPoseArray,
            'object_pose_torso', 
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self, msg: ObjPoseArray):
        if not msg.data:
            self.get_logger().info("No objects received")
            return

        self.get_logger().info(f"Received {len(msg.data)} objects:")
        for obj in msg.data:
            pose = obj.pose_stamped.pose
            if obj.object_name == "bottle":
                self.get_logger().info(
                    f"Object: {obj.object_name}, "
                    f"Position -> x:{pose.position.x:.2f}, y:{pose.position.y:.2f}, z:{pose.position.z:.2f}, "
                )


def main(args=None):
    rclpy.init(args=args)
    node = ObjPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
