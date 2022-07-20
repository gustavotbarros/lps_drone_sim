from time import sleep
import rclpy
from rclpy.node import Node

from px4_msgs.msg import *
from std_msgs.msg import *

import math

class RoutePublisher(Node):
    def __init__(self):
        super().__init__('route_publisher')

        # publishers:
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/vehicle_command/in", 10)
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "fmu/offboard_control_mode/in", 10)

        # subscribers:
        self.vehicle_odometry_sub_ = self.create_subscription(VehicleOdometry, "fmu/vehicle_odometry/out", self.process_vehicle_odometry, 10)
        self.route_sp_counter_sub_ = self.create_subscription(UInt16, "/com/setpoints_counter", self.process_setpoints_counter, 10)
        self.start_route_sub_ = self.create_subscription(Bool, "/com/start_route", self.process_start_request, 10)
        self.route_setpoints_sub0_ = self.create_subscription(TrajectorySetpoint, "/com/route_setpoints0", self.setpoint0_callback, 10)
        self.route_setpoints_sub1_ = self.create_subscription(TrajectorySetpoint, "/com/route_setpoints1", self.setpoint1_callback, 10)
        self.route_setpoints_sub2_ = self.create_subscription(TrajectorySetpoint, "/com/route_setpoints2", self.setpoint2_callback, 10)
        self.route_setpoints_sub3_ = self.create_subscription(TrajectorySetpoint, "/com/route_setpoints3", self.setpoint3_callback, 10)
        self.route_setpoints_sub4_ = self.create_subscription(TrajectorySetpoint, "/com/route_setpoints4", self.setpoint4_callback, 10)

        # parameters and local variables
        self.setpoints0 = [0.0, 0.0, 0.0, 0.0]
        self.setpoints1 = [0.0, 0.0, 0.0, 0.0]
        self.setpoints2 = [0.0, 0.0, 0.0, 0.0]
        self.setpoints3 = [0.0, 0.0, 0.0, 0.0]
        self.setpoints4 = [0.0, 0.0, 0.0, 0.0]
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        self.setpoints_counter = 0
        self.setpoints_max = 0

        self.initial_position_x = 0.0
        self.initial_position_y = 0.0

        # initialize parameters
        self.timestamp_ = 0
        timer_period = 0.1
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.start_route = False
        self.destination_check = False

    def timer_callback(self):
        # arms the vehicle:
        if self.start_route == True:
            self.arm()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
            
            # send the setpoints:
            if self.setpoints_counter == 0:
                self.x = self.initial_position_x
                self.y = self.initial_position_y
                self.z = self.setpoints0[2]
                self.yaw = float("NaN")
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                self.destination_check = True
            
            # send the setpoints:
            if self.setpoints_counter == 1:
                self.x = self.setpoints0[0]
                self.y = self.setpoints0[1]
                self.z = self.setpoints0[2]
                self.yaw = self.setpoints0[3]
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                self.destination_check = True
            
            # send the setpoints:
            if self.setpoints_counter == 2:
                self.x = self.setpoints1[0]
                self.y = self.setpoints1[1]
                self.z = self.setpoints1[2]
                self.yaw = self.setpoints1[3]
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                self.destination_check = True

            # send the setpoints:
            if self.setpoints_counter == 3:
                self.x = self.setpoints2[0]
                self.y = self.setpoints2[1]
                self.z = self.setpoints2[2]
                self.yaw = self.setpoints2[3]
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                self.destination_check = True

            # send the setpoints:
            if self.setpoints_counter == 4:
                self.x = self.setpoints3[0]
                self.y = self.setpoints3[1]
                self.z = self.setpoints3[2]
                self.yaw = self.setpoints3[3]
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                self.destination_check = True

            # send the setpoints:    
            if self.setpoints_counter == 5:
                self.x = self.setpoints4[0]
                self.y = self.setpoints4[1]
                self.z = self.setpoints4[2]
                self.yaw = self.setpoints4[3]
                self.publish_offboard_control_mode()
                self.publish_trajectory_setpoint()
                self.destination_check = True

        # lands the vehicle:
        if self.setpoints_counter == self.setpoints_max + 1:
            self.publish_offboard_control_mode()
            sleep(1)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.start_route = False
            self.setpoints_counter = 0

    # callback for number of setpoints
    def process_setpoints_counter(self, msg):
        self.setpoints_max = msg.data
    
    # triggers the route start
    def process_start_request(self, msg):
        self.start_route = msg.data

    # reads the input setpoint inputs from the console    
    def setpoint0_callback(self, setpoints):
        self.setpoints0[0] = setpoints.x
        self.setpoints0[1] = setpoints.y
        self.setpoints0[2] = setpoints.z
        self.setpoints0[3] = setpoints.yaw

    # reads the input setpoint inputs from the console
    def setpoint1_callback(self, setpoints):
        self.setpoints1[0] = setpoints.x
        self.setpoints1[1] = setpoints.y
        self.setpoints1[2] = setpoints.z
        self.setpoints1[3] = setpoints.yaw

    # reads the input setpoint inputs from the console
    def setpoint2_callback(self, setpoints):
        self.setpoints2[0] = setpoints.x
        self.setpoints2[1] = setpoints.y
        self.setpoints2[2] = setpoints.z
        self.setpoints2[3] = setpoints.yaw

    # reads the input setpoint inputs from the console
    def setpoint3_callback(self, setpoints):
        self.setpoints3[0] = setpoints.x
        self.setpoints3[1] = setpoints.y
        self.setpoints3[2] = setpoints.z
        self.setpoints3[3] = setpoints.yaw
    
    # reads the input setpoint inputs from the console
    def setpoint4_callback(self, setpoints):
        self.setpoints4[0] = setpoints.x
        self.setpoints4[1] = setpoints.y
        self.setpoints4[2] = setpoints.z
        self.setpoints4[3] = setpoints.yaw

    # publish the setpoints from the control interface
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.yaw = self.yaw
        
        self.trajectory_setpoint_publisher_.publish(msg)

    # reads the odometry from the vehicle to determine if the setpoints have been reached
    def process_vehicle_odometry(self,odom):
        self.initial_position_x = odom.x
        self.initial_position_y = odom.y 

        if self.destination_check == True:
            self.target_x = self.x
            self.target_y = self.y
            self.target_z = self.z
                                                                                
            dt_x = self.target_x - odom.x
            dt_y = self.target_y - odom.y
            dt_z = self.target_z - odom.z

            dt_dist = (math.sqrt((dt_x)**2+(dt_y)**2+(dt_z)**2))
            if dt_dist < 0.1:
                self.setpoints_counter += 1
                self.destination_check = False
            
    # publish the offboard control mode (position is active)
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        msg.timestamp = self.timestamp_
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
        msg.timestamp = self.timestamp_
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
