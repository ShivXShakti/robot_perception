#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from dualarm_custom_msgs.msg import ObjPoseArray, ObjPose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import time
from depthai_ros_msgs.msg import TrackDetection2DArray 


class ObjectPoseTransformer(Node):
    def __init__(self):
        super().__init__('object_pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(
            TrackDetection2DArray,
            '/color/yolo_Spatial_tracklets',
            self.listener_callback,
            10
        )
        self.publisher = self.create_publisher(ObjPoseArray, 'object_pose_torso', 10)
        self.object_classes = ["person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
                            "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
                            "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
                            "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
                            "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
                            "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
                            "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
                            "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
                            "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"]

        self.get_logger().info("ObjectPoseTransformer YOLO initialized")

    def listener_callback(self, msg):
        msg_array = ObjPoseArray()
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            class_id = int(result.hypothesis.class_id)
            score = result.hypothesis.score
            pose = result.pose.pose
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.header.frame_id = msg.header.frame_id
            pose_stamped.pose = pose

            try:
                transformed_pose = self.tf_buffer.transform(
                    pose_stamped,
                    'torso',
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                transformed_pose.header.stamp = self.get_clock().now().to_msg()
                obj = ObjPose()
                obj.object_name = f"{self.object_classes[class_id]}"
                obj.pose_stamped.pose = transformed_pose.pose
                msg_array.data.append(obj)
            except Exception as e:
                self.get_logger().warn(f"Could not transform pose: {e}")
        print(f"====================Detected Objects===================:\n {msg_array.data}")
        self.publisher.publish(msg_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
