from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import *
from std_msgs.msg import *

import math

class RoutePublisher(Node):
    def __init__(self):
        super().__init__('route_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # publishers:
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/in/vehicle_command", qos_profile)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "fmu/in/offboard_control_mode", qos_profile)
        self.setpoint_control_publisher_ = self.create_publisher(UInt16, "/com/setpoint_control",qos_profile)

        # subscribers:
        self.vehicle_odometry_sub_ = self.create_subscription(VehicleOdometry, "fmu/out/vehicle_odometry", self.process_vehicle_odometry, qos_profile)
        self.route_sp_counter_sub_ = self.create_subscription(UInt16, "/com/setpoints_counter", self.process_setpoints_counter, qos_profile)
        self.start_route_sub_ = self.create_subscription(Bool, "/com/start_route", self.process_start_request, qos_profile)
        self.land_request_sub_ = self.create_subscription(Bool, "/com/land_request", self.process_land_request, qos_profile)
        self.route_setpoints_sub0_ = self.create_subscription(Float32MultiArray, "/com/route_setpoints", self.setpoints_callback, qos_profile)

        # parameters and local variables
        self.setpoints = []
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.setpoints_counter = 0
        self.setpoints_max = 0

        self.i = 4

        self.initial_position_x = 0.0
        self.initial_position_y = 0.0
        self.initial_position_z = 0.0

        # initialize parameters
        self.timestamp_ = 0
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.start_route = False
        self.destination_check = False
        self.land_request = False

    def timer_callback(self):
        # arms the vehicle:
        if self.start_route == True and self.land_request == False:
            self.arm()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))

            self.publish_offboard_control_mode()
        
            self.x = self.setpoints[0]
            self.y = self.setpoints[1]
            self.z = self.setpoints[2]
            self.yaw = self.setpoints[3]
            
            self.publish_trajectory_setpoint()
            print(self.x,self.y,self.z)
            self.destination_check = True
                
        # lands the vehicle:
        if self.land_request == True:
            self.publish_offboard_control_mode()
            sleep(1)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.start_route = False
            self.setpoints_counter = 0
            self.setpoints = []
            self.land_request = False

    # callback for number of setpoints
    def process_setpoints_counter(self, msg):
        self.setpoints_max = msg.data
    
    # triggers the route start
    def process_start_request(self, msg):
        self.start_route = msg.data
    
    # triggers the land request
    def process_land_request(self, msg):
        self.land_request = msg.data
        print(self.land_request)

    # reads the input setpoint inputs from the console    
    def setpoints_callback(self, setpoints):
        self.setpoints = setpoints.data

    # publish the setpoints from the control interface
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        msg.position[0] = self.x
        msg.position[1] = self.y
        msg.position[2] = self.z
        msg.yaw = self.yaw
        
        self.trajectory_setpoint_publisher_.publish(msg)

    # reads the odometry from the vehicle to determine if the setpoints have been reached
    def process_vehicle_odometry(self,odom):
        self.initial_position_x = odom.position[0]
        self.initial_position_y = odom.position[1]
        self.initial_position_z = odom.position[2]

        if self.destination_check == True:
            self.target_x = self.x
            self.target_y = self.y
            self.target_z = self.z
                                                                                
            dt_x = self.target_x - odom.position[0]
            dt_y = self.target_y - odom.position[1]
            dt_z = self.target_z - odom.position[2]

            dt_dist = (math.sqrt((dt_x)**2+(dt_y)**2+(dt_z)**2))
            print(dt_dist)
            if dt_dist < 0.1 and self.land_request == False:
                msg = UInt16()
                msg.data = 4
                self.setpoint_control_publisher_.publish(msg)
                self.destination_check = False
            
    # publish the offboard control mode (position is active)
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_control_mode_publisher_.publish(msg)

    # send a command to arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    # send a command to desarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)

    # publish the vehicle commands (arm, disarm, etc.)
    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

# main:
def main(args=None):
    rclpy.init()
    route_publisher = RoutePublisher()
    rclpy.spin(route_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()