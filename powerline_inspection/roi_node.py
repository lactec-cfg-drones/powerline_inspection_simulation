#!/usr/bin/env python3 
import numpy as np 
import rclpy 
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection2DArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation as R



class RoiNode(Node): 

    def __init__(self):
        super().__init__('roi_node')

    
        self.last_yolo_bbox = None
        

        # camera frame 
        self.target_frame = 'camera_front_link' # Frame da câmera
        

        # camera config (from the urdf model)
        self.fx = 701.93225
        self.fy = 701.93225
        self.cx = 799.5
        self.cy = 599.5
        self.width = 1600
        self.height = 1200

        
        # tf configs 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        # subscribers 
        self.lidar_sub = self.create_subscription(PointCloud2, '/front_laser/point_cloud', self.lidar_callback, 10)
        self.yolo_sub = self.create_subscription(Detection2DArray, '/yolo/detections', self.yolo_callback, 10)
        
        # publishers
        self.roi_pub = self.create_publisher(PointCloud2, '/perception/point_cloud', 10) 

        self.lastest_detections = []





    def yolo_callback(self, msg): 
        if len(msg.detections) > 0: 
            self.last_yolo_bbox = msg.detections[0].bbox
        else:
            self.last_yolo_bbox = None






    # transform lidar frame to camera frame 
    def apply_transform(self, points, transform):
        
        q = transform.transform.rotation
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z])
        
        # P_cam = R * P_lidar + T
        return (points @ rot_matrix.T) + translation






    def lidar_callback(self, msg): 
        if self.last_yolo_bbox is None:
            return


        # get tf data 
        try:
            source_frame = msg.header.frame_id
            trans = self.tf_buffer.lookup_transform(self.target_frame, source_frame, rclpy.time.Time())
        except Exception:
            return
        
        
        # convert point cloud to numpy array and discart nan values 
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts_lidar = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
        pts_lidar = pts_lidar[np.all(np.isfinite(pts_lidar), axis=1)]
        if pts_lidar.size == 0: return


        # frames transforms 
        pts_cam = self.apply_transform(pts_lidar, trans)

       
        # ajust optical frame  
        # at gazebo, x=ahead, for the ros projection we need to put z ahead with a 90 degree rotation
        # P_optical_x = -P_cam_y
        # P_optical_y = -P_cam_z
        # P_optical_z =  P_cam_x
        x_opt = -pts_cam[:, 1]
        y_opt = -pts_cam[:, 2]
        z_opt =  pts_cam[:, 0]


        # depth filter to discart the readings close to the drone
        valid_mask = z_opt > 1.5 
        x_opt, y_opt, z_opt = x_opt[valid_mask], y_opt[valid_mask], z_opt[valid_mask]
        pts_final = pts_cam[valid_mask]


        if z_opt.size == 0: return


        # image projection 
        u = (self.fx * x_opt / z_opt) + self.cx
        v = (self.fy * y_opt / z_opt) + self.cy


        # get region of interest from yolo
        b = self.last_yolo_bbox
        u_min, u_max = b.center.position.x - b.size_x/2, b.center.position.x + b.size_x/2
        v_min, v_max = b.center.position.y - b.size_y/2, b.center.position.y + b.size_y/2


        mask_u = (u >= u_min) & (u <= u_max)
        mask_v = (v >= v_min) & (v <= v_max)
        

        roi_mask = mask_u & mask_v & (z_opt < 50.0)
        roi_points = pts_final[roi_mask]


        print(f"U min/max: {u.min():.1f}/{u.max():.1f} | V min/max: {v.min():.1f}/{v.max():.1f}")


        if len(roi_points) > 0:
            # publish data 
            roi_msg = pc2.create_cloud_xyz32(msg.header, roi_points)
            roi_msg.header.frame_id = self.target_frame
            self.roi_pub.publish(roi_msg)
            self.get_logger().info(f"ALVO LOCALIZADO: {len(roi_points)} pontos na torre.")
        
        else:
            self.get_logger().warn(f"ROI empty. U_m: {np.mean(u):.1f}, V_m: {np.mean(v):.1f}")





def main(args=None):
    rclpy.init(args=args)
    node = RoiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()