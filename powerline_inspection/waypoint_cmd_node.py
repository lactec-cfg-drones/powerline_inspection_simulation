#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition

class WaypointCmd(Node): 

    def __init__(self):
        super().__init__('waypoint_cmd_node')


        self.offboard_setpoint_counter = 0
        self.vehicle_status = VehicleStatus()
        self.vehicle_local_position = VehicleLocalPosition()
        self.waypoints = [          # x, y, z, theta
            [0.0, 0.0, -20.0, 1.57], 
            [-60.0, 0.0, -20.0, - 1.57],
        ]

        self.current_wp_index = 0
        self.acceptance_radius = 1.0


        # QoS config 
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )



        # publishers 
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)


        # subscribers 
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_position_callback, qos_profile)


        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Waypoint Commander Node Started! Sending commands...")

    
    
    
    def local_position_callback(self, msg):
        self.vehicle_local_position = msg



    # get current vehicle status
    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg



    # publish offboard control mode
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.offboard_control_mode_pub.publish(msg)




    # publish a command to the drone 
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.vehicle_cmd_pub.publish(msg)


    
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        target_wp = self.waypoints[self.current_wp_index]
        msg.position = [float(target_wp[0]), float(target_wp[1]), float(target_wp[2])]
        msg.yaw = target_wp[3]
        msg.timestamp = int(self.get_clock().now().nanoseconds/1000)

        self.trajectory_setpoint_pub.publish(msg)





    def check_waypoint_reached(self):

        if self.current_wp_index < len(self.waypoints) - 1: 
            target = self.waypoints[self.current_wp_index]

            dx = target[0] - self.vehicle_local_position.x
            dy = target[1] - self.vehicle_local_position.y
            dz = target[2] - self.vehicle_local_position.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)

            # If we are close enough, move to the next point
            if distance < self.acceptance_radius:
                self.get_logger().info(f"Waypoint {self.current_wp_index} reached! Moving to next.")
                self.current_wp_index += 1
        
        else: 
            self.get_logger().info('At last point')




    # arm drone 
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')




    # switch to offboard mode
    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to Offboard mode")




    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        self.check_waypoint_reached()

        if self.offboard_setpoint_counter == 10: 
            self.engage_offboard_mode()
            self.arm()
        
        if self.offboard_setpoint_counter < 11: 
            self.offboard_setpoint_counter += 1
    



def main(args=None):
    rclpy.init(args=args)
    node = WaypointCmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

