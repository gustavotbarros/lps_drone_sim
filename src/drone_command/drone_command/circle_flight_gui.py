import rclpy
from rclpy import executors
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import threading
import numpy as np
from time import sleep

from px4_msgs.msg import *
from std_msgs.msg import * 

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from drone_control.circle_flight import CircleFlight

class CircleFlightGUI(QWidget):

    # top-level layout for circle flight publisher
    def __init__(self):
        super().__init__()
        self.setFont(QFont("Arial", 14))
        self.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, False)
        self.setWindowTitle('Setpoint Control')

        self.destination_reached = False

        self.circle_set_layout = QFormLayout()

        self.sp_label = QLabel(self)
        self.sp_label.setText("Starting Conditions:")

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
        self.z_entry.setText("1.5")

        self.yaw_label = QLabel(self)
        self.yaw_label.setText("Yaw [°]:")
        self.yaw_entry = QLineEdit(self)
        self.yaw_entry.setText("0")

        self.circle_label = QLabel(self)
        self.circle_label.setText("Circle:")

        self.radius_label = QLabel(self)
        self.radius_label.setText("R [m]:")
        self.radius_entry = QLineEdit(self)
        self.radius_entry.setText("0.8")

        self.turns_label = QLabel(self)
        self.turns_label.setText("Turns [-]:")
        self.turns_entry = QLineEdit(self)
        self.turns_entry.setText("2.0")

        self.status_label = QLabel(self)
        self.status_label.setFont(QFont("Arial", 8))
        self.status_label.setText("Waiting for inputs.")

        self.start_button = QPushButton("Start", self)
        self.start_button.clicked.connect(self.start_drone)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.clicked.connect(self.stop_drone)

        self.return_button = QPushButton("Return to menu", self)
        self.return_button.clicked.connect(self.returnButton)

        self.circle_set_layout.addRow(self.sp_label)
        self.circle_set_layout.addRow(self.x_label, self.x_entry)
        self.circle_set_layout.addRow(self.y_label, self.y_entry)
        self.circle_set_layout.addRow(self.z_label, self.z_entry)
        self.circle_set_layout.addRow(self.yaw_label, self.yaw_entry)
        self.circle_set_layout.addRow(self.circle_label)
        self.circle_set_layout.addRow(self.radius_label, self.radius_entry)
        self.circle_set_layout.addRow(self.turns_label, self.turns_entry)

        self.circle_set_layout.addRow(self.start_button)
        self.circle_set_layout.addRow(self.stop_button)

        self.circle_set_layout.addRow(self.status_label) 
        self.circle_set_layout.addRow(self.return_button)

        self.setLayout(self.circle_set_layout)

        self.pub_sub_init()

    # returns to main menu
    def returnButton(self):
        msg = Bool()
        msg.data = True
        self.main_menu_publisher_.publish(msg)
        self.stop_drone()
        sleep(0.25)
        self.close()
        sleep(0.25)
        self.destroy_nodes()
        
    # setpoint - circle requester
    def start_drone(self):
        msg = Bool()
        msg.data = True
        self.circle_request_publisher_.publish(msg)

        try:
            if float(self.x_entry.text()) <= 1.8 and float(self.x_entry.text()) >= -1.8 \
                and float(self.y_entry.text()) <= 1.8 and float(self.y_entry.text()) >= -1.8 \
                and float(self.z_entry.text()) <= 1.8 and float(self.z_entry.text()) >= -1.8 \
                and float(self.radius_entry.text()) <= 1.7 and float(self.radius_entry.text()) >= -1.7:
                circ = Float32MultiArray()

                circ.data = [float(self.x_entry.text()),float(self.y_entry.text()),float(self.z_entry.text()) * -1.0,np.deg2rad(float(self.yaw_entry.text()))\
                    , float(self.radius_entry.text()),float(self.turns_entry.text())]

                print(circ.data)
               
                self.status_label.setText("Doing circle...")
                self.circle_flight_gui_.publish(circ)

            else:
                self.status_label.setText("Inputs are not valid. Please stay at the margin.")

        except ValueError:
            self.status_label.setText("Inputs are not valid.")

    # setpoint - stop drone
    def stop_drone(self):
        msg = Bool()
        msg.data = True
        self.stop_request_publisher_.publish(msg)

    # vehicle status callback for controlling the gui
    def process_vehicle_status(self, flag):    
        if flag.arming_state == 2:
            self.start_button.setEnabled(False)
            self.status_label.setText("Route started.")
        else:
            self.stop_button.setEnabled(True)
            self.start_button.setEnabled(True)
            self.status_label.setText("Waiting for inputs.")

    # publishers and subscribers on thread executor
    def pub_sub_init(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.circ_pub = rclpy.create_node("circle_flight_gui")
        self.circle_flight_gui_ = self.circ_pub.create_publisher(Float32MultiArray, "/com/circle_info", qos_profile)
        self.circle_request_publisher_ = self.circ_pub.create_publisher(Bool, "/com/circle_request", qos_profile)
        self.stop_request_publisher_ = self.circ_pub.create_publisher(Bool, "/com/stop_request", qos_profile)
        
        self.main_menu_pub = rclpy.create_node("main_menu_publisher")
        self.main_menu_publisher_ = self.main_menu_pub.create_publisher(Bool, "/com/main_menu", qos_profile)

        self.odom_sub = rclpy.create_node("process_vehicle_odometry")
        self.status_sub = rclpy.create_node("process_vehicle_status")
        self.vehicle_status_sub_ = self.status_sub.create_subscription(VehicleStatus, "fmu/out/vehicle_status", self.process_vehicle_status, qos_profile)

        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.odom_sub)
        self.executor.add_node(self.circ_pub)
        self.executor.add_node(self.main_menu_pub)
        self.executor.add_node(self.status_sub)
        self.circle_flight = CircleFlight()
        self.executor.add_node(self.circle_flight)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
    
    def destroy_nodes(self):
        self.circ_pub.destroy_node()
        self.main_menu_pub.destroy_node()
        self.odom_sub.destroy_node()
        self.status_sub.destroy_node()
        self.circle_flight.destroy_node()
        self.executor.shutdown()