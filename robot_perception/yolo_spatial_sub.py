#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from dualarm_custom_msgs.msg import ObjPoseArray, ObjPose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import time
from depthai_ros_msgs.msg import TrackDetection2DArray 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class ObjectPoseTransformer(Node):
    def __init__(self):
        super().__init__('object_pose_transformer')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_img = None
        self.bridge = CvBridge()

        self.create_subscription(
            TrackDetection2DArray,
            '/color/yolo_Spatial_tracklets',
            self.listener_callback,
            10
        )
        self.create_subscription(
            Image,
            '/color/image',
            self.image_callback,
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
    def image_callback(self, msg: Image):
        # Convert ROS image to OpenCV
        self.current_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    
    def listener_callback(self, msg):
        if self.current_img is None:
            return
        img = self.current_img.copy()
        for detection in msg.detections:
            if not detection.results:
                continue
            result = detection.results[0]
            class_id = int(result.hypothesis.class_id)
            score = result.hypothesis.score

            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)
            x1 = int(cx - w / 2)
            y1 = int(cy - h / 2)
            x2 = int(cx + w / 2)
            y2 = int(cy + h / 2)

            # Get image center
            img_h, img_w, _ = img.shape
            cx_img = img_w // 2
            cy_img = img_h // 2

            # Draw image center (blue)
            cv2.circle(img, (cx_img, cy_img), 6, (255, 0, 0), -1)

            if self.object_classes[class_id] == "bottle":
                print(f"====================Detected Objects===================:\n {result.pose.pose.position}")
                cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"ID:{self.object_classes[class_id]} ({score:.2f})", 
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 2)
                Ttor_cam = np.array([[0.001, -0.707,  0.707,  0.078],
                                    [-1.000,  0.000,  0.0,  0.0],
                                    [0.0, -0.707, -0.707,  0.107],
                                    [0.0,  0.0,  0.0,  1.0]])

                Tcam_scr = np.array([[1, 0, 0, 0],
                                     [0, -1, 0, 0.0],
                                     [0, 0, -1, result.pose.pose.position.z],
                                     [0,0,0,1]])
                
                Tscr_ob = np.array([[1, 0, 0, result.pose.pose.position.x],
                                     [0, 1, 0, result.pose.pose.position.y],
                                     [0, 0, 1, 0],
                                     [0,0,0,1]])
                Tcam_ob = Ttor_cam@Tcam_scr @ Tscr_ob
                ##print(f"Tcam_ob: {Tcam_ob}") 
        cv2.imshow("YOLO Detections", img)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
