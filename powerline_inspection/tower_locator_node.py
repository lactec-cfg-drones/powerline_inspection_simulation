import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

import numpy as np 
import os 

import torch
import torch.nn as nn
import torch.nn.functional as F

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

import sensor_msgs_py.point_cloud2 as pc2 

from tf2_ros import TransformException 
from tf2_ros import Buffer 
from tf2_ros.transform_listener import TransformListener 
import tf2_geometry_msgs



WEIGHTS_PATH = '../scripts/model_results/tower_pointnet2_weights.pth'


# model architecture ---------------------------------------------------------------------------------------------------------------------------------
class PointNetSetAbstraction(nn.Module):
    
    def __ini__(self, in_channel, mlp):
        super(PointNetSetAbstraction, self).__init__()
        self.mlp_convs = nn.ModuleList()
        self.mlp_bns = nn.ModuleList()
        last_channel = in_channel # channels are the features per point 

        for out_channel in mlp: 
            self.mlp_convs.append(nn.Conv2d(last_channel, out_channel, 1))
            self.mlp_bns.append(nn.BatchNorm2d(out_channel))
            last_channel = out_channel



    def foward(self, points): 
        x = points.unsqueeze(-1)
        for i, conv in enumerate(self.mlp_convs):
            bn = self.mlp_bns[i]
            x = F.relu(bn(conv(x)))
        return torch.max(x,2)[0]
    


class PointNet2Cls(nn.Module):
    def __init__(self, num_class=2):
        super(PointCloud2, self).__init__()
        self.sa1 = PointNetSetAbstraction(in_channel = 3, mlp = [64, 64, 128])
        self.sa2 = PointNetSetAbstraction(in_channel = 128, mlp = [128, 128, 256])
        self.sa3 = PointNetSetAbstraction(in_channel = 256, mlp = [256, 512, 1024])

        self.fc1 = nn.Linear(1024, 512)
        self.bn1 = nn.BatchNorm1d(512)
        self.fc2 = nn.Linear(512, 256)
        self.bn2 = nn.BatchNorm1d(256)
        self.fc3 = nn.Linear(256, num_class)
    



    def foward(self, x): 
        x = self.sa1(x)
        x = self.sa2(x)
        x = self.sa3(x)
        x = x.view(-1, 1024)
        x = F.relu(self.bn1(self.fc1(x)))
        x = F.relu(self.bn2(self.fc2(x)))
        return F.log_softmax(self.fc3(x), -1) 
    



# ros2 node ------------------------------------------------------------------------------------------------------------------------------------------------
class TowerLocator (Node):
    def __init__(self): 
        super().__init__('tower_locator_node')
        

        # neural network detection parameters
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = PointNet2Cls(num_class = 2).to(self.device)
        
        if os.path.exists(WEIGHTS_PATH):
            self.model.load_state_dict(torch.load(WEIGHTS_PATH, map_location = self.device))
            self.model.eval()
            self.get_logger().info('PointNet++ is ready!')
        else:
            self.get_logger().error(f'Weights not found at {WEIGHTS_PATH}')


        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # qos policy
        sensor_qos_config = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT, 
            durability = QoSDurabilityPolicy.VOLATILE, 
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5,
        )

        # subscribers
        self.roi_sub = self.create_subscription(PointCloud2, '/perception/tower/roi_points', self.points_callback, sensor_qos_config)

        # publishers
        self.tower_pose_pub = self.create_publisher(PoseStamped, '/tower_pose', 10)



    
    def points_callback(self, msg):
        # convert point cloud to numpy
        gen = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        points = np.array([[p[0], p[1], p[2]] for p in gen], dtype = np.float32)

        if len(points) < 50: 
            return

        # points normalization
        centroid = np.mean(points, axis=0)
        pts_norm = points - centroid
        max_dist = np.max(np.sqrt(np.sum(pts_norm**2, axis=1)))
        if max_dist > 0: pts_norm /= max_dist

        # resampling to 1024 samples
        choice = np.random.choice(len(pts_norm), 1024, replace=True)
        pts_input = pts_norm[choice, :]
        input_tensor = torch.from_numpy(pts_input).float().transpose(1,0).unsqueeze(0).to(self.device)

        # inference
        with torch.no_grad():
            pred = self.model(input_tensor)
            confidence = torch.exp(pred)
            class_id = torch.argmax(confidence, dim=1).item()
            prob = confidence[0][class_id].item()

        # if valid, publish tower location
        if class_id == 0 and prob > 0.8: 
            self.get_tower_pose_world_frame(self, centroid, msg.header)

        else: 
            self.get_logger().warn("Low confidence of the target") 

        

    

    def get_tower_pose_world_frame(self, msg, centroid, header): 
        try: 

            local_pose = PoseStamped()
            local_pose.header = header
            local_pose.pose.position.x = float(centroid[0])
            local_pose.pose.position.y = float(centroid[1])
            local_pose.pose.position.z = float(centroid[2])


            transform = self.tf_buffer.lookup_transform('world', header.frame_id, header.stamp, timeout=rclpy.duration.Duration(seconds=0.05))

            world_pose_msg = tf2_geometry_msgs.do_transform_pose(local_pose.pose, transform)

            final_msg = PoseStamped()
            final_msg.header.stamp = header.stamp 
            final_msg.header.frame_id = 'world'
            final_msg.pose = world_pose_msg

            self.tower_pose_pub.publish(final_msg)
            self.get_logger().info('Published tower location')

        except TransformException as ex: 
            self.get_logger().error(f'TF error: {ex}')




def main():
    rclpy.init()
    node = TowerLocator()
    rclpy.spin(node) 
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()


