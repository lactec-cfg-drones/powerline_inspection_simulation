#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import transforms3d as tf3d 

class DroneTFBroadcaster(Node):

    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        # Initializing the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers for bridged topics
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom,
            10)
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.handle_imu,
            10)

        self.get_logger().info('Drone TF Broadcaster Node Started Successfully')

    def handle_imu(self, msg):
        # Optional: You can extract heading (yaw) here if your odom lacks orientation
        pass

    def handle_odom(self, msg):
        now = self.get_clock().now().to_msg() #


        t_odom_footprint = TransformStamped()
        t_odom_footprint.header.stamp = now
        t_odom_footprint.header.frame_id = 'odom'
        t_odom_footprint.child_frame_id = 'base_link'
        
        t_odom_footprint.transform.translation.x = msg.pose.pose.position.x
        t_odom_footprint.transform.translation.y = msg.pose.pose.position.y
        t_odom_footprint.transform.translation.z = 0.0
        t_odom_footprint.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t_odom_footprint) #

def main(args=None):
    rclpy.init(args=args)
    node = DroneTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down node.')
    finally:
        node.destroy_node()
        try: 
            rclpy.shutdown()
        except:
            pass
if __name__ == '__main__':
    main()