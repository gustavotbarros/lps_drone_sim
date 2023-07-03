from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math
import numpy as np

from px4_msgs.msg import *
from std_msgs.msg import *

class CircleFlight(Node):
    def __init__(self):
        super().__init__('circle_flight')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # publishers:
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "fmu/in/offboard_control_mode", qos_profile)
        self.circle_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/in/vehicle_command", qos_profile)
        self.landed_publisher_ = self.create_publisher(Bool, "/com/landed", qos_profile)

        # subscribers:
        self.circle_request_sub_ = self.create_subscription(Bool, "/com/circle_request", self.process_circle_request, qos_profile)
        self.stop_request_sub_ = self.create_subscription(Bool, "/com/stop_request", self.process_stop_request, qos_profile)
        self.circle_setpoint_sub_ = self.create_subscription(Float32MultiArray, "/com/circle_info", self.process_circle_setpoint, qos_profile)
        self.vehicle_odometry_sub_ = self.create_subscription(VehicleOdometry, "fmu/out/vehicle_odometry", self.process_vehicle_odometry, qos_profile)

        # parameters and local variables
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.yawspeed = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.acceleration = [0.0, 0.0, 0.0]
        self.jerk = [0.0, 0.0, 0.0]
        self.thrust = [0.0, 0.0, 0.0]
        self.timestamp = 0
        self.radius = 0.0
        self.sp = [0.0,0.0,0.0,0.0]
        self.initial_position_x = 0.0
        self.initial_position_y = 0.0
        self.initial_position_z = 0.0
        self.setpoints_counter = 0
        self.turns = 0
        self.theta_max = 0.0

        # flags used to control the system
        self.circle_request = False
        self.armed = False
        self.stop_request = False
        self.landed = True
        self.takeoff_request = False
        self.destination_check = False
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.dt = timer_period
        self.theta = 0.0
        self.omega = 0.5

    # reads the timestamp
    def timer_callback(self):
        # arms the vehicle:
        if self.circle_request == True:
            self.arm()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))

            self.theta_max = 6.3*self.turns

            self.publish_offboard_control_mode()

            self.x = self.sp[0]
            self.y = self.sp[1]
            self.z = self.sp[2]
            self.yaw = self.sp[3]

            self.publish_trajectory_setpoint()

            self.destination_check = True

            # positioning on starting point:
            if self.setpoints_counter == 1:
                self.x = self.sp[0] + self.radius
                self.y = self.sp[1]
                self.z = self.sp[2]
                self.yaw = self.sp[3]

                self.publish_trajectory_setpoint()

                self.destination_check = True
            
            # draw a circle:
            if self.setpoints_counter == 2:
                self.destination_check = False
          
                self.x = float(self.sp[0]) + float(self.radius)*np.cos(self.theta)
                self.y = float(self.sp[1]) + float(self.radius)*np.sin(self.theta)
                self.z = float(self.z)
                self.yaw = np.deg2rad(self.yaw)

                self.publish_trajectory_setpoint()

                self.theta = self.theta + self.omega*self.dt

        # lands the vehicle:
        if self.circle_request == True and self.stop_request == True or self.theta_max < self.theta:
            
            self.x = float(self.radius + self.x)*np.cos(self.theta)
            self.y = float(self.radius + self.y)*np.sin(self.theta)
            self.z = float(self.z)
            self.yaw = np.deg2rad(self.yaw)

            self.publish_trajectory_setpoint()

            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.circle_request = False
            self.stop_request = False
            self.setpoints_counter = 0
            self.theta = 0.0
            self.omega = 0.5

    # reads the input setpoint inputs from the console
    def process_circle_setpoint(self, circ): 
        self.sp[0] = circ.data[0]
        self.sp[1] = circ.data[1]
        self.sp[2] = circ.data[2]
        self.sp[3] = circ.data[3]
        self.radius = circ.data[4] + self.x
        self.turns = circ.data[5]
 
    
    # reads the odometry from the vehicle to determine if the setpoints have been reached
    def process_vehicle_odometry(self, odom):      
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
            if dt_dist < 0.1:
                self.setpoints_counter += 1
                self.destination_check = False
                        
    # triggers the arm procedure
    def process_circle_request(self, msg):
        self.circle_request = msg.data

    # triggers the land procedures
    def process_stop_request(self, msg):
        self.stop_request = msg.data

    # send a command to arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    # send a command to desarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)

    # publish the offboard control mode (position and feed forward velocity are active)
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)
        
    # publish the setpoints from the control interface
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position[0] = self.x
        msg.position[1] = self.y
        msg.position[2] = self.z
        msg.yaw = self.yaw

        self.circle_setpoint_publisher_.publish(msg)

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

def main(args=None):
    rclpy.init()
    circle_flight = CircleFlight()
    rclpy.spin(circle_flight)
    rclpy.shutdown()

if __name__ == "__main__":
    main()