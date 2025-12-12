#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import time


class ObjectPoseTransformer(Node):
    def __init__(self):
        super().__init__('object_pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PoseStamped,
            'object_pose_camera',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(PoseStamped, 'object_pose_torso', 10)
        self.get_logger().info("ObjectPoseTransformer initialized")

    def listener_callback(self, msg: PoseStamped):
        try:
            if not isinstance(msg, PoseStamped):
                self.get_logger().warn(f"Received message is not PoseStamped: {type(msg)}")
                return

            msg.header.stamp = rclpy.time.Time().to_msg()

            transformed_pose = self.tf_buffer.transform(
                msg,
                'torso_base',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(transformed_pose)

            p = transformed_pose.pose.position
            self.get_logger().info(
                f"Pose wrt torso: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}"
            )

        except Exception as e:
            self.get_logger().warn(f"Could not transform pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
