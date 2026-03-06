#!/usr/bin/env python3 

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class yolo_detection_node(Node):

    def __init__(self):
        super().__init__('yolo_detection_node')
        self.bridge = CvBridge()

        # load yolo config
        self.get_logger().info('Loading YOLO11 model')

        weights = '/home/leticia/.pyenv/runs/detect/tower_identification/yolo11_test/weights/best.pt'
        self.model = YOLO(weights)

        self.get_logger().info('Model loaded successfully')


        # create sub
        self.image_sub = self.create_subscription(Image, '/camera_front/image', self.image_callback, 10)

    


    def image_callback(self, msg):
        try: 
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            h, w = cv_image.shape[:2]


            # inference 
            results = self.model(cv_image, verbose=False)


            for box in results[0].boxes:
                # get id and name of the identified class
                class_id = int(box.cls[0])
                class_name = self.model.names[class_id]
                confidence = float(box.conf[0])


                if class_name == 'transmission tower' and confidence > 0.50:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])

                    # draw the ROI 
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
                    cv2.putText(cv_image, f'Torre: {confidence:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.6, (255, 0, 0), 2)


                    margin_search = 300
                    roi_esq_x1, roi_esq_x2 = max(0, x1 - margin_search), x1
                    roi_dir_x1, roi_dir_x2 = x2, min(w, x2 + margin_search)

                    cv2.rectangle(cv_image, (roi_esq_x1, y1), (roi_esq_x2, y2), (0, 255, 0), 2)
                    cv2.rectangle(cv_image, (roi_dir_x1, y1), (roi_dir_x2, y2), (0, 255, 0), 2)

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



