#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from dualarm_custom_msgs.msg import ObjPoseArray, ObjPose


class ObjectPosePublisher(Node):
    def __init__(self):
        super().__init__('object_pose_publisher')
        self.publisher_ = self.create_publisher(ObjPoseArray, 'object_pose_torso', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg_array = ObjPoseArray()
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rclpy.time.Time().to_msg()
        pose_stamped.header.frame_id = 'camera_rgb_optical_frame'

        pose_stamped.pose.position.x = 0.6
        pose_stamped.pose.position.y = 0.18
        pose_stamped.pose.position.z = -0.45
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        obj = ObjPose()
        obj.object_name = f"bottle"
        obj.pose_stamped.pose = pose_stamped.pose
        msg_array.data.append(obj)

        self.publisher_.publish(msg_array)
        self.get_logger().info('Publishing object pose in camera frame')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
