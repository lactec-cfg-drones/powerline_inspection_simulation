#!/usr/bin/env python3 
import os
import rclpy 
import numpy as np 
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.node import Node
from std_msgs.msg import Empty
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
        self.target_frame = 'camera_front_link' 

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
        self.save_cmd_sub = self.create_subscription(Empty, '/cmd/save_roi', self.save_cmd_callback, 10)
        
        # publishers
        self.roi_pub = self.create_publisher(PointCloud2, '/perception/point_cloud', 10) 

        # Config para salvar dados
        self.save_requested = False
        self.data_dir = 'dataset_pointnet'
        self.save_count = 0
        
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)







    def save_cmd_callback(self, msg): 
        self.save_requested = True
        self.get_logger().info('✅ Comando de salvamento recebido! Capturando próximo frame...')






    def yolo_callback(self, msg): 
        if len(msg.detections) > 0: 
            self.last_yolo_bbox = msg.detections[0].bbox
        else:
            self.last_yolo_bbox = None






    def apply_transform(self, points, transform):
        # Garantir que não há NaNs nos pontos antes da conta
        points = points[np.all(np.isfinite(points), axis=1)]
        
        q = transform.transform.rotation
        rot_matrix = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        t = transform.transform.translation
        translation = np.array([t.x, t.y, t.z])
        
        return (points @ rot_matrix.T) + translation






    def lidar_callback(self, msg): 
        if self.last_yolo_bbox is None:
            return

        try:
            source_frame = msg.header.frame_id
           
            trans = self.tf_buffer.lookup_transform(
                self.target_frame, 
                source_frame, 
                rclpy.time.Time()) 
        except Exception:
            return
        
        # conversion and remove nans 
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        pts_lidar = np.array([[p[0], p[1], p[2]] for p in gen], dtype=np.float32)
        if pts_lidar.size == 0: return


        # transform to he camera frame
        pts_cam = self.apply_transform(pts_lidar, trans)


        # ajust optical frame 
        x_opt = -pts_cam[:, 1]
        y_opt = -pts_cam[:, 2]
        z_opt =  pts_cam[:, 0]


        # descart the samples cllose to the drone 
        valid_mask = z_opt > 2.0 
        x_opt, y_opt, z_opt = x_opt[valid_mask], y_opt[valid_mask], z_opt[valid_mask]
        pts_final = pts_cam[valid_mask]


        if z_opt.size == 0: return


        # image projection
        u = (self.fx * x_opt / z_opt) + self.cx
        v = (self.fy * y_opt / z_opt) + self.cy


        # get the roi from yolo (more a margin of 10%)
        b = self.last_yolo_bbox
        pad = 1.10 
        u_min, u_max = b.center.position.x - (b.size_x/2)*pad, b.center.position.x + (b.size_x/2)*pad
        v_min, v_max = b.center.position.y - (b.size_y/2)*pad, b.center.position.y + (b.size_y/2)*pad


        # depth of the window
        spatial_mask = (u >= u_min) & (u <= u_max) & (v >= v_min) & (v <= v_max)


        if not np.any(spatial_mask):
            return


        z_tower = z_opt[spatial_mask]
        median_depth = np.median(z_tower)
        

        thickness = 8.0 
        depth_mask = (z_opt >= (median_depth - thickness/2)) & (z_opt <= (median_depth + thickness/2))
        
        roi_mask = spatial_mask & depth_mask
        roi_points = pts_final[roi_mask]


        # publish and save data 
        if len(roi_points) > 0:
            roi_msg = pc2.create_cloud_xyz32(msg.header, roi_points)
            roi_msg.header.frame_id = self.target_frame
            self.roi_pub.publish(roi_msg)
            

            if len(roi_points) < 50:
                self.get_logger().warn(f"Poucos pontos capturados ({len(roi_points)}). Verifique o alinhamento.")
            else:
                self.get_logger().info(f"ALVO LOCALIZADO: {len(roi_points)} pontos.")


            if self.save_requested: 
                # data normalization
                centroid = np.mean(roi_points, axis=0)
                pts_normalized = roi_points - centroid


                # normalize scale 
                max_dist = np.max(np.sqrt(np.sum(pts_normalized**2, axis=1)))
                if max_dist > 0:
                    pts_normalized = pts_normalized / max_dist

                timestamp = self.get_clock().now().nanoseconds
                filename = os.path.join(self.data_dir, f"tower_{timestamp}.npy")
                np.save(filename, pts_normalized)
                

                self.save_count += 1
                self.get_logger().info(f'💾 Nuvem #{self.save_count} salva e normalizada! Arquivo: {filename}')
                self.save_requested = False
        else:
            # Feedback de debug if the ROI is empty 
            self.get_logger().warn("ROI is empty.")





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