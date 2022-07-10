import rclpy
from rclpy.node import Node

import math

from px4_msgs.msg import *
from std_msgs.msg import *

class SetpointPublisher(Node):
    def __init__(self):
        super().__init__('setpoint_publisher')

        # publishers:
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "fmu/offboard_control_mode/in", 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/trajectory_setpoint/in", 10)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/vehicle_command/in", 10)
        self.destination_reached_publisher_ = self.create_publisher(Bool, "/com/destination_reached", 10)
        self.landed_publisher_ = self.create_publisher(Bool, "/com/landed", 10)

        # subscribers:
        self.timesync_sub_ = self.create_subscription(Timesync, "fmu/timesync/out", self.timesync_callback, 10)
        self.arm_request_sub_ = self.create_subscription(Bool, "/com/arm_request", self.process_arm_request, 10)
        self.land_request_sub_ = self.create_subscription(Bool, "/com/land_request", self.process_land_request, 10)
        self.trajectory_setpoint_sub_ = self.create_subscription(TrajectorySetpoint, "/com/use_setpoint", self.process_trajectory_setpoint, 10)
        self.vehicle_odometry_sub_ = self.create_subscription(VehicleOdometry, "fmu/vehicle_odometry/out", self.process_vehicle_odometry, 10)


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

        # flags used to control the system
        self.arm_request = False
        self.armed = False
        self.land_request = False
        self.landed = True
        self.takeoff_request = False
        self.destination_reached = False

    # reads the timestamp
    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    # reads the input setpoint inputs from the console
    def process_trajectory_setpoint(self, traj): 
        self.x = traj.x
        self.y = traj.y
        self.z = traj.z
        self.yaw = traj.yaw
        self.vx = traj.vx
        self.vy = traj.vy
        self.vz = traj.vz
        self.acceleration = traj.acceleration
        self.jerk = traj.jerk
        self.thrust = traj.thrust
    
    # reads the odometry from the vehicle to determine if the setpoints have been reached
    def process_vehicle_odometry(self, odom):
        # arms the vehicle:
        if self.arm_request == True:
            self.arm()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, float(1), float(6))
            self.landed = False
            self.arm_request = False
            self.armed = True
        
        # send the setpoints:
        if self.landed == False and self.armed == True:
            self.publish_offboard_control_mode()
            self.publish_trajectory_setpoint()
            
            self.target_x = self.x
            self.target_y = self.y
            self.target_z = self.z
                                                                            
            dt_x = self.target_x - odom.x
            dt_y = self.target_y - odom.y
            dt_z = self.target_z - odom.z

            self.landed = False
            dt_dist = (math.sqrt((dt_x)**2+(dt_y)**2+(dt_z)**2))
            # check if destination is reached:
            if dt_dist < 0.1:
                msg = Bool()

                msg.data = True

                self.destination_reached_publisher_.publish(msg)
                
        # lands the vehicle:
        if self.armed == True and self.land_request == True:
            self.publish_offboard_control_mode()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
            self.land_request = False
            self.armed = False
            self.landed = True
        
    # triggers the arm procedure
    def process_arm_request(self, msg):
        self.arm_request = msg.data

    # triggers the land procedures
    def process_land_request(self, msg):
        self.land_request = msg.data

    # send a command to arm the vehicle
    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)

    # send a command to desarm the vehicle
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)

    # publish the offboard control mode (position and feed forward velocity are active)
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_control_mode_publisher_.publish(msg)
        
    # publish the setpoints from the control interface
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp
        msg.x = self.x
        msg.y = self.y
        msg.z = self.z
        msg.yaw = self.yaw
        msg.vx = self.vx
        msg.vy = self.vy
        msg.vz = self.vz
        msg.acceleration = self.acceleration
        msg.jerk = self.jerk
        msg.thrust = self.thrust

        self.trajectory_setpoint_publisher_.publish(msg)

    # publish the vehicle commands (arm, disarm, etc.)
    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()

        msg.timestamp = self.timestamp
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
    setpoint_publisher = SetpointPublisher()
    rclpy.spin(setpoint_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
