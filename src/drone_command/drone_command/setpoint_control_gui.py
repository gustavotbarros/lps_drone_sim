import rclpy
from rclpy import executors

import threading
import numpy as np
from time import sleep

from px4_msgs.msg import *
from std_msgs.msg import * 

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from drone_control.setpoint_publisher import SetpointPublisher

class SetpointControlGUI(QWidget):

    # top-level layout for setpoint and velocity feed forward publisher
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Setpoint Control")
        self.setFont(QFont("Arial", 14))
        self.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, False)

        combo_layout = QVBoxLayout()
        self.setLayout(combo_layout)

        self.page_combo = QComboBox()
        self.page_combo.addItems(["Position Setpoints", "Velocity Setpoints"])
        self.page_combo.activated.connect(self.switchPage)

        self.stacked_layout = QStackedLayout()

        self.return_button = QPushButton("Return to menu", self)
        self.return_button.clicked.connect(self.returnButton)

        self.destination_reached = False

        # layout for setpoint publisher
        self.traj_set_gui = QWidget()
        self.traj_set_layout = QFormLayout()

        self.x_label = QLabel(self)
        self.x_label.setText("X [m]:")
        self.x_entry = QLineEdit(self)
        self.x_entry.setText("0.0")

        self.y_label = QLabel(self)
        self.y_label.setText("Y [m]:")
        self.y_entry = QLineEdit(self)
        self.y_entry.setText("0.0")

        self.z_label = QLabel(self)
        self.z_label.setText("Z [m]:")
        self.z_entry = QLineEdit(self)
        self.z_entry.setText("2.5")

        self.yaw_label = QLabel(self)
        self.yaw_label.setText("Yaw [°]:")
        self.yaw_entry = QLineEdit(self)
        self.yaw_entry.setText("90")

        self.status_label = QLabel(self)
        self.status_label.setFont(QFont("Arial", 8))
        self.status_label.setText("Waiting for inputs.")

        self.traj_set_gui.setLayout(self.traj_set_layout)
        self.stacked_layout.addWidget(self.traj_set_gui)

        self.arm_button = QPushButton("Arm", self)
        self.arm_button.clicked.connect(self.arm_drone)

        self.fly_button = QPushButton("Fly", self)
        self.fly_button.clicked.connect(self.fly_to_position)

        self.land_button = QPushButton("Land", self)
        self.land_button.clicked.connect(self.land)

        self.traj_set_layout.addRow(self.x_label, self.x_entry)
        self.traj_set_layout.addRow(self.y_label, self.y_entry)
        self.traj_set_layout.addRow(self.z_label, self.z_entry)
        self.traj_set_layout.addRow(self.yaw_label, self.yaw_entry)

        self.traj_set_layout.addRow(self.arm_button)
        self.traj_set_layout.addRow(self.fly_button)
        self.traj_set_layout.addRow(self.land_button)

        self.traj_set_layout.addRow(self.status_label)  

        # layout for velocity feed forward publisher
        self.vel_set_gui = QWidget()
        self.vel_set_layout = QFormLayout()

        self.vx_label = QLabel(self)
        self.vx_label.setText("Vx [m/s]:")
        self.vx_entry = QLineEdit(self)
        self.vx_entry.setText("0.0")

        self.vy_label = QLabel(self)
        self.vy_label.setText("Vy [m/s]:")
        self.vy_entry = QLineEdit(self)
        self.vy_entry.setText("0.0")

        self.vz_label = QLabel(self)
        self.vz_label.setText("Vz [m/s]:")
        self.vz_entry = QLineEdit(self)
        self.vz_entry.setText("0.0")

        self.status2_label = QLabel(self)
        self.status2_label.setFont(QFont("Arial", 8))
        self.status2_label.setText("Drone will fly constantly at these velocities.")

        self.vel_set_gui.setLayout(self.vel_set_layout)
        self.stacked_layout.addWidget(self.vel_set_gui)

        self.set_button = QPushButton("Set", self)
        self.set_button.clicked.connect(self.velocity_setpoint)

        self.vel_set_layout.addRow(self.vx_label, self.vx_entry)
        self.vel_set_layout.addRow(self.vy_label, self.vy_entry)
        self.vel_set_layout.addRow(self.vz_label, self.vz_entry)

        self.vel_set_layout.addRow(self.set_button)

        self.vel_set_layout.addRow(self.status2_label)

        # adds the combo box and the stacked layout to the top-level layout
        combo_layout.addWidget(self.page_combo)
        combo_layout.addLayout(self.stacked_layout)
        combo_layout.addWidget(self.return_button)

        self.pub_sub_init()

    # switchs between setpoint and velocity publisher
    def switchPage(self):
        self.stacked_layout.setCurrentIndex(self.page_combo.currentIndex())

    # returns to main menu
    def returnButton(self):
        msg = Bool()
        msg.data = True
        self.main_menu_publisher_.publish(msg)
        self.close()
        sleep(0.25)
        self.destroy_nodes()
        
    # setpoint - arm requester
    def arm_drone(self):
        msg = Bool()
            
        msg.data = True
        self.arm_request_publisher_.publish(msg)

        self.status_label.setText("Waiting for inputs.")

    # setpoint - fly requester
    def fly_to_position(self):
        try:
            traj = TrajectorySetpoint()

            traj.x = float(self.x_entry.text())
            traj.y = float(self.y_entry.text())
            traj.z = float(self.z_entry.text()) * -1.0
            traj.yaw = np.deg2rad(float(self.yaw_entry.text()))

            self.status_label.setText("Flying to setpoint...")
            self.trajectory_setpoint_gui_.publish(traj)

        except ValueError:
            self.status_label.setText("Inputs are not valid.")

    # velocity - fly requester
    def velocity_setpoint(self):
        try:
            traj = TrajectorySetpoint()

            traj.x = float("NaN")
            traj.y = float("NaN")
            traj.y = float("NaN")
            traj.z = float("NaN")
            traj.yaw = float("NaN")
            traj.yawspeed = float("NaN")
            traj.vx = float(self.vx_entry.text())
            traj.vy = float(self.vy_entry.text())
            traj.vz = float(self.vz_entry.text()) * -1.0
            traj.acceleration = [float("NaN"), float("NaN"), float("NaN")]
            traj.jerk = [float("NaN"), float("NaN"), float("NaN")]
            traj.thrust = [float("NaN"), float("NaN"), float("NaN")]

            self.trajectory_setpoint_gui_.publish(traj)

        except ValueError:
            self.status2_label.setText("Inputs are not valid.")

    # setpoint - land requester
    def land(self):
        msg = Bool()
        msg.data = True
        self.land_request_publisher_.publish(msg)

    # vehicle status callback for controlling the gui
    def process_vehicle_status(self, flag):    
        # checks if armed:
        if flag.arming_state == 1:
            self.set_button.setEnabled(False)
            self.land_button.setEnabled(False)
            self.arm_button.setEnabled(True)
            self.status_label.setText("The drone must be armed.")
            self.status2_label.setText("The drone must be flying.")
        elif flag.arming_state == 2:
            self.set_button.setEnabled(True)
            self.land_button.setEnabled(True)
            self.arm_button.setEnabled(False)
            self.status2_label.setText("Drone will be feed forward with these velocities.")

        # checks for fly button:
        if flag.nav_state == 18 and flag.arming_state == 2:
            self.fly_button.setEnabled(False)
            self.status_label.setText("Landing...")
        elif flag.nav_state == 18 and flag.arming_state == 1:
            self.fly_button.setEnabled(False)
        elif flag.nav_state == 14 and flag.arming_state == 1:
            self.fly_button.setEnabled(False)
        elif flag.nav_state == 4 and flag.arming_state == 1:
            self.fly_button.setEnabled(False)
        else:
            self.fly_button.setEnabled(True)

    # checks if destination is reached for gui control
    def process_destination(self, msg):
        self.destination_reached = msg.data
        if self.destination_reached == True:
            self.status_label.setText("Destination reached.")
        else:
            self.status_label.setText("Waiting for inputs.")

    # publishers and subscribers on thread executor
    def pub_sub_init(self):
        self.traj_pub = rclpy.create_node("trajectory_setpoint_gui")
        self.trajectory_setpoint_gui_ = self.traj_pub.create_publisher(TrajectorySetpoint, "/com/use_setpoint", 10)
        self.arm_request_publisher_ = self.traj_pub.create_publisher(Bool, "/com/arm_request", 10)
        self.land_request_publisher_ = self.traj_pub.create_publisher(Bool, "/com/land_request", 10)
        
        self.main_menu_pub = rclpy.create_node("main_menu_publisher")
        self.main_menu_publisher_ = self.main_menu_pub.create_publisher(Bool, "/com/main_menu", 10)

        self.odom_sub = rclpy.create_node("process_vehicle_odometry")
        self.destination_reached_sub_ = self.odom_sub.create_subscription(Bool, "/com/destination_reached", self.process_destination, 10)
        self.status_sub = rclpy.create_node("process_vehicle_status")
        self.vehicle_status_sub_ = self.status_sub.create_subscription(VehicleStatus, "fmu/vehicle_status/out", self.process_vehicle_status, 10)

        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.odom_sub)
        self.executor.add_node(self.traj_pub)
        self.executor.add_node(self.main_menu_pub)
        self.executor.add_node(self.status_sub)
        self.setpoint_publisher = SetpointPublisher()
        self.executor.add_node(self.setpoint_publisher)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
    
    def destroy_nodes(self):
        self.traj_pub.destroy_node()
        self.main_menu_pub.destroy_node()
        self.odom_sub.destroy_node()
        self.status_sub.destroy_node()
        self.setpoint_publisher.destroy_node()
        self.executor.shutdown()









