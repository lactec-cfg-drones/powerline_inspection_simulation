#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
import transforms3d as tf3d 

class DroneTFBroadcaster(Node):

    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        self.tf_broadcaster = TransformBroadcaster(self)

        # subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.handle_odom, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.handle_imu, 10)
        self.gps_sub = self.create_subscription(NavSatFix, '/navsat', self.handle_gps, 10)

        self.origin_lat = None
        self.origin_lon = None

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
        t_odom_footprint.transform.translation.z = msg.pose.pose.position.z 
        t_odom_footprint.transform.rotation = msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t_odom_footprint) 
    


    def handle_gps(self, msg): 
        # define the first data as the origin of the frame world
        if self.origin_lat is None: 
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude

            self.get_logger().info(f'Origin of the frame world defined at: {self.origin_lat}, {self.origin_lon}')



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