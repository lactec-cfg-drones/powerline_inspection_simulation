#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition, VehicleAttitude

class WaypointCmd(Node): 

    def __init__(self):
        super().__init__('waypoint_cmd_node')

        # State Variables
        self.vehicle_status = VehicleStatus()
        self.local_pos = VehicleLocalPosition()
        self.vehicle_attitude = VehicleAttitude()
        self.offboard_setpoint_counter = 0
        
        # We only store the World X coordinate for this phase
        self.target_world_x = None 
        self.target_world_y = None
        self.takeoff_height = -20.0 

        self.yaw = None

        # QoS for PX4 Compatibility
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            durability=DurabilityPolicy.TRANSIENT_LOCAL, 
            history=HistoryPolicy.KEEP_LAST, 
            depth=1
        )

        # Publishers
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.vehicle_cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        
        # Subscribers
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, qos)
        self.pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.pos_cb, qos)
        self.att_sub = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.att_cb, qos)
        self.goal_sub = self.create_subscription(PointStamped, '/drone/goal_pose', self.goal_callback, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("X-AXIS DEBUG: Moving North/South. Y and Z are locked to current pose.")

    def goal_callback(self, msg):
        """Calculates World X using Body-to-World rotation."""
        if not any(self.vehicle_attitude.q): 
            return

        # 1. Extract current Yaw from Drone Quaternion
        q = self.vehicle_attitude.q
        self.yaw = math.atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2]**2 + q[3]**2))

        # 2. Rotate Relative LiDAR X (Forward) and Y (Lateral) into World X (North)
        # Assuming LiDAR frame: X is forward, Y is left.
        rel_x = msg.point.x
        rel_y = msg.point.y # Flip if sensor Y is positive-left
        
        # Transformation: World_X = Drone_X + (rel_x * cos(yaw) - rel_y * sin(yaw))
        world_rel_x = rel_x * math.cos(self.yaw) - rel_y * math.sin(self.yaw)
        world_rel_y = rel_x * math.sin(self.yaw) + rel_y * math.cos(self.yaw)

        self.target_world_x = self.local_pos.x + world_rel_x
        self.target_world_y = self.local_pos.y + world_rel_y
      
        
        if self.offboard_setpoint_counter % 20 == 0:
            self.get_logger().info(f"YAW: {math.degrees(self.yaw):.1f} | TARGET WORLD X: {self.target_world_x:.2f} | TARGET WORLD Y: {self.target_world_y:.2f}")

    def timer_callback(self):
        self.publish_offboard_control_mode()

        # Startup Sequence
        if (self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD or 
            self.vehicle_status.arming_state != VehicleStatus.ARMING_STATE_ARMED):
            
            if self.offboard_setpoint_counter >= 10:
                self.engage_offboard_mode()
                self.arm()
            self.offboard_setpoint_counter += 1
            return

        msg = TrajectorySetpoint()
        
        # MISSION LOGIC: X-AXIS ONLY
        if self.target_world_x is not None:
            target_x = float(self.target_world_x)
        else:
            target_x = self.local_pos.x # Stay at current X if no target seen

        # "IGNORE" Y and Z by feeding back current position
        target_y = self.target_world_y
        target_z = self.local_pos.z

        msg.position = [target_x, target_y, target_z]
        msg.yaw = self.yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        max_vel = float(0.5) 
        msg.velocity = [max_vel, max_vel, max_vel]
        self.setpoint_pub.publish(msg)

        if self.offboard_setpoint_counter % 50 == 0:
            self.get_logger().info(f"POS: X={self.local_pos.x:.1f}, Y={self.local_pos.y:.1f}, Z={self.local_pos.z:.1f}")
        
        self.offboard_setpoint_counter += 1

    # Callback Helpers
    def pos_cb(self, msg): self.local_pos = msg
    def status_cb(self, msg): self.vehicle_status = msg
    def att_cb(self, msg): self.vehicle_attitude = msg
    
    def publish_offboard_control_mode(self):
        m = OffboardControlMode(position=True, timestamp=int(self.get_clock().now().nanoseconds/1000))
        self.offboard_mode_pub.publish(m)

    def arm(self): self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
    def engage_offboard_mode(self): self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def publish_command(self, command, param1=0.0, param2=0.0):
        m = VehicleCommand(command=command, param1=param1, param2=param2, target_system=1, 
                           target_component=1, source_system=1, source_component=1, from_external=True)
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(m)

def main(args=None):
    rclpy.init(args=args); n = WaypointCmd(); rclpy.spin(n); n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()