import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import math

from px4_msgs.msg import *
from std_msgs.msg import *

class SetpointPublisher(Node):
    def __init__(self):
        super().__init__('setpoint_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        # publishers:
        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, "fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "fmu/in/vehicle_command", qos_profile)
        self.destination_reached_publisher_ = self.create_publisher(Bool, "/com/destination_reached", qos_profile)
        self.landed_publisher_ = self.create_publisher(Bool, "/com/landed", qos_profile)

        # subscribers:
        #self.timesync_sub_ = self.create_subscription(Timesync, "fmu/out/timesync", self.timesync_callback, 10)
        self.arm_request_sub_ = self.create_subscription(Bool, "/com/arm_request", self.process_arm_request, qos_profile)
        self.land_request_sub_ = self.create_subscription(Bool, "/com/land_request", self.process_land_request, qos_profile)
        self.trajectory_setpoint_sub_ = self.create_subscription(TrajectorySetpoint, "/com/use_setpoint", self.process_trajectory_setpoint, qos_profile)
        self.vehicle_odometry_sub_ = self.create_subscription(VehicleOdometry, "fmu/out/vehicle_odometry", self.process_vehicle_odometry, qos_profile)


        # parameters and local variables
        self.position = [0.0, 0.0, 0.0]
        self.yaw = 0.0
        self.yawspeed = 0.0
        self.velocity = [0.0, 0.0, 0.0]
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
    #def timesync_callback(self, msg):
    #    self.timestamp = msg.timestamp

    # reads the input setpoint inputs from the console
    def process_trajectory_setpoint(self, traj): 
        self.position[0] = traj.position[0]
        self.position[1] = traj.position[1]
        self.position[2] = traj.position[2]
        self.yaw = traj.yaw
        self.velocity[0] = traj.velocity[0]
        self.velocity[1] = traj.velocity[1]
        self.velocity[2] = traj.velocity[2]
        self.acceleration = traj.acceleration
        self.jerk = traj.jerk
    
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
            
            self.target_x = self.position[0]
            self.target_y = self.position[1]
            self.target_z = self.position[2]
                                                                            
            dt_x = self.target_x - odom.position[0]
            dt_y = self.target_y - odom.position[1]
            dt_z = self.target_z - odom.position[2]

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
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = False

        self.offboard_control_mode_publisher_.publish(msg)
        
    # publish the setpoints from the control interface
    def publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()

        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position[0] = self.position[0]
        msg.position[1] = self.position[1]
        msg.position[2] = self.position[2]
        msg.yaw = self.yaw
        msg.velocity[0] = self.velocity[0]
        msg.velocity[1] = self.velocity[1]
        msg.velocity[2] = self.velocity[2]
        msg.acceleration = self.acceleration
        msg.jerk = self.jerk
        #msg.thrust = self.thrust

        self.trajectory_setpoint_publisher_.publish(msg)

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
    setpoint_publisher = SetpointPublisher()
    rclpy.spin(setpoint_publisher)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
