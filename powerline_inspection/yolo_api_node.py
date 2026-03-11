#!/usr/bin/env python3 
import numpy as np
import math 

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO





class yolo_detection_node(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')
        self.bridge = CvBridge()

        # State memory for Sensor Fusion
        self.latest_box = None
        self.latest_image_width = None


        # Load YOLO model
        self.get_logger().info('Loading YOLO11 model')
        weights = '/home/leticia/.pyenv/runs/detect/tower_identification/yolo11_test/weights/best.pt'
        self.model = YOLO(weights)
        self.get_logger().info('Model loaded successfully')


        # Subscriptions
        self.image_sub = self.create_subscription(Image, '/camera_front/image', self.image_callback, 10)


        # Publishers
        self.yolo_detection_pub = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        self.detection_array = Detection2DArray()
        self.detection = Detection2D()
        self.hypothesis = ObjectHypothesisWithPose()



    def image_callback(self, msg):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w = cv_image.shape[:2]

            # keep the original header
            self.detection_array.header = msg.header

            # YOLO Inference
            results = self.model(cv_image, verbose=False)


            for box in results[0].boxes:
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])


                if class_name == 'transmission tower' and confidence > 0.50:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # bounding box
                    self.detection.header = msg.header
                    self.detection.bbox.center.position.x = float(x1 + x2)/2
                    self.detection.bbox.center.position.y = float(y1 + y2)/2
                    self.detection.bbox.size_x = float(x2 - x1)
                    self.detection.bbox.size_y = float(y2 - y1)

                    # class information
                    self.hypothesis.hypothesis.class_id = str(class_id)
                    self.hypothesis.hypothesis.score = confidence
                    self.detection.results.append(self.hypothesis)

                    self.detection_array.detections.append(self.detection)

                    # Visualization
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
                    cv2.putText(cv_image, f'Tower: {confidence:.2f}', (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # publish yolo detection
            self.yolo_detection_pub.publish(self.detection_array)

            cv2.imshow('Vision', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'ERROR at cv_bridge: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = yolo_detection_node()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()