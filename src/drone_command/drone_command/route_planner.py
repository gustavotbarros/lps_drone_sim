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

from drone_control.route_publisher import RoutePublisher

class RoutePlannerGUI(QWidget):

    # top-level layout for the route planner
    def __init__(self):
        super().__init__()
        self.setFont(QFont("Arial", 14))
        self.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.setWindowFlag(Qt.WindowMaximizeButtonHint, False)
        self.setWindowTitle('Route Planner')

        self.setpoints = {}
        self.setpoints_counter = 0
        self.setpoint_control = 0
        self.land_request = False

        self.setpoints_list = QListWidget()
        self.setpoints_list.setFixedHeight(150)
        self.setpoints_list.setFixedWidth(300)
        
        self.route_label = QLabel(self)
        self.route_label.setText("Route:")
        self.route_label.setFont(QFont("Arial", 14, weight=QFont.Bold))

        self.status_label = QLabel(self)
        self.status_label.setFont(QFont("Arial", 8))

        self.edit_stations_button = QPushButton("Add Station Setpoint")
        self.edit_stations_button.clicked.connect(self.edit_stations)

        self.clear_route_button = QPushButton("Clear Route")
        self.clear_route_button.clicked.connect(self.clear_route)

        self.return_menu_button = QPushButton("Return to Menu")
        self.return_menu_button.clicked.connect(self.return_menu)

        self.start_route_button = QPushButton("Start Route")
        self.start_route_button.pressed.connect(self.start_route)

        self.rp_gui_layout = QFormLayout()

        self.rp_gui_layout.addRow(self.route_label)
        self.rp_gui_layout.addRow(self.setpoints_list)
        self.rp_gui_layout.addRow(self.start_route_button)
        self.rp_gui_layout.addRow(self.edit_stations_button)
        self.rp_gui_layout.addRow(self.clear_route_button)
        self.rp_gui_layout.addRow(self.return_menu_button)
        self.rp_gui_layout.addRow(self.status_label)

        self.setLayout(self.rp_gui_layout)

        self.pub_sub_init()

    # return to main menu
    def return_menu(self):
        msg = Bool()
        msg.data = True
        self.main_menu_publisher_.publish(msg)
        
        self.close()
        sleep(0.25)
        self.destroy_nodes()

    # starts the route
    def start_route(self):
        msg = UInt16()

        msg.data = self.setpoints_counter

        self.setpoints_counter_publisher_.publish(msg)

        msg = Bool()

        msg.data = True

        self.start_route_publisher_.publish(msg)

        self.land_request = False
        self.setpoints_counter = self.setpoints_list.count()

        setpoints = []
        for sublist in self.setpoints.values():
            for item in sublist:
                setpoints.append(item)

        set = Float32MultiArray()
                
        set.data = [0.0,0.0,0.0,0.0]
        set.data[0] = setpoints[0]
        set.data[1] = setpoints[1]
        set.data[2] = setpoints[2]
        set.data[3] = setpoints[3]

        self.route_setpoints_publisher_.publish(set)

        print(setpoints)
        print(self.setpoint_control)
        print(self.setpoints_counter) 

    # layout for edit setpoint window
    def edit_stations(self):
        self.edit_window = QWidget()
        self.editWLayout = QFormLayout()
        self.edit_window.setWindowTitle("New Station")
        self.edit_window.setWindowFlag(Qt.WindowCloseButtonHint, False)
        self.edit_window.setWindowFlag(Qt.WindowMaximizeButtonHint, False)

        self.x_label = QLabel(self)
        self.x_label.setText("X [m]:")
        self.x_entry = QLineEdit(self)

        self.y_label = QLabel(self)
        self.y_label.setText("Y [m]:")
        self.y_entry = QLineEdit(self)

        self.z_label = QLabel(self)
        self.z_label.setText("Z [m]:")
        self.z_entry = QLineEdit(self)

        self.yaw_label = QLabel(self)
        self.yaw_label.setText("Yaw [°]:")
        self.yaw_entry = QLineEdit(self)

        self.edit_save_button = QPushButton("Save")
        self.edit_save_button.clicked.connect(self.edit_save)
        
        self.edit_quit_button = QPushButton("Quit")
        self.edit_quit_button.clicked.connect(self.edit_quit)

        self.edit_window_status_label = QLabel(self)
        self.edit_window_status_label.setText("")

        self.edit_window.setLayout(self.editWLayout)

        self.editWLayout.addRow(self.x_label, self.x_entry)
        self.editWLayout.addRow(self.y_label, self.y_entry)
        self.editWLayout.addRow(self.z_label, self.z_entry)
        self.editWLayout.addRow(self.yaw_label, self.yaw_entry)
        self.editWLayout.addRow(self.edit_quit_button, self.edit_save_button)
        self.editWLayout.addRow(self.edit_window_status_label)

        self.edit_window.show()

    # function to define the setpoints and how many it will be
    def edit_save(self):
        try:
            if self.setpoints_counter > 25:
                self.edit_save_button.setEnabled(False)
                self.edit_window_status_label.setText("Maximum number of setpoints reached.")
            else:
                self.setpoint = QListWidgetItem()
                self.setpoint_data = (float(self.x_entry.text()),float(self.y_entry.text()),(float(self.z_entry.text())),float(self.yaw_entry.text()))
                self.setpoint.setData(Qt.UserRole, self.setpoint_data)
                self.edit_window_status_label.setText("")
                self.setpoint.setText("SP - X:" + str(self.setpoint_data[0]) + " Y:" + str(self.setpoint_data[1]) + " Z:" + str(self.setpoint_data[2]) + " Yaw:" + str(self.setpoint_data[3]) + "°")

                self.setpoints["setpoint" + str(self.setpoints_counter)] = [float(self.x_entry.text()),float(self.y_entry.text()),-(float(self.z_entry.text())),np.deg2rad(float(self.yaw_entry.text()))]
                print(list(self.setpoints.values()))
                self.setpoints_counter += 1

                self.setpoints_list.insertItem(Qt.UserRole, self.setpoint)

        except ValueError:
            self.edit_window_status_label.setText("Inputs are not valid.")
    
    # closes the edit setpoint window
    def edit_quit(self):
        self.edit_window.close()

    # clears the setpoints from the route
    def clear_route(self):
        self.setpoints_list.clear()
        self.setpoints_counter = 0
        self.setpoints.clear()

    # vehicle status callback for controlling the gui
    def process_vehicle_status(self, flag):
        if flag.arming_state == 2:
            self.start_route_button.setEnabled(False)
            self.clear_route_button.setEnabled(False)
            self.edit_stations_button.setEnabled(False)
            self.status_label.setText("Route started.")
        else:
            self.start_route_button.setEnabled(True)
            self.clear_route_button.setEnabled(True)
            self.edit_stations_button.setEnabled(True)
            self.status_label.setText("Waiting for inputs.")
    
    def process_setpoint_control(self,msg):
        if self.land_request == False and self.setpoint_control < (self.setpoints_counter*4):
            self.setpoint_control = self.setpoint_control + msg.data

            setpoints = []
            for sublist in self.setpoints.values():
                for item in sublist:
                    setpoints.append(item)

            print(setpoints)
            print(self.setpoint_control)
            print(self.setpoints_counter) 
            
            set = Float32MultiArray()
                    
            set.data = [0.0,0.0,0.0,0.0]
            set.data[0] = setpoints[self.setpoint_control]
            set.data[1] = setpoints[self.setpoint_control+1]
            set.data[2] = setpoints[self.setpoint_control+2]
            set.data[3] = setpoints[self.setpoint_control+3]

            self.route_setpoints_publisher_.publish(set)
            
        if self.setpoint_control == (self.setpoints_counter*4):
            self.land_request = True

            print(self.setpoint_control)

            self.setpoints_counter = 0
            self.setpoint_control = 0

            msg = Bool()
            msg.data = True

            self.land_request_publisher_.publish(msg)


    # publishers and subscribers on thread executor
    def pub_sub_init(self):
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.route_setpoints_pub = rclpy.create_node("route_setpoints_gui")
        self.setpoints_counter_publisher_ = self.route_setpoints_pub.create_publisher(UInt16, "/com/setpoints_counter", qos_profile)
        self.start_route_publisher_ = self.route_setpoints_pub.create_publisher(Bool, "/com/start_route", qos_profile)
        self.route_setpoints_publisher_ = self.route_setpoints_pub.create_publisher(Float32MultiArray, "/com/route_setpoints", qos_profile)
        self.land_request_publisher_ = self.route_setpoints_pub.create_publisher(Bool,"com/land_request", qos_profile)
        self.setpoint_control_sub_ = self.route_setpoints_pub.create_subscription(UInt16, "/com/setpoint_control", self.process_setpoint_control,qos_profile)

        self.main_menu_pub = rclpy.create_node("main_menu_publisher")
        self.main_menu_publisher_ = self.main_menu_pub.create_publisher(Bool, "/com/main_menu", qos_profile)

        self.vehicle_status_sub = rclpy.create_node("process_vehicle_status")
        self.vehicle_status_sub_ = self.vehicle_status_sub.create_subscription(VehicleStatus, "fmu/out/vehicle_status", self.process_vehicle_status, qos_profile)

        self.executor = executors.MultiThreadedExecutor()
        self.executor.add_node(self.main_menu_pub)
        self.executor.add_node(self.route_setpoints_pub)
        self.executor.add_node(self.vehicle_status_sub)
        self.route_publisher = RoutePublisher()  
        self.executor.add_node(self.route_publisher)

        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
    
    def destroy_nodes(self):
        self.main_menu_pub.destroy_node()
        self.route_setpoints_pub.destroy_node()
        self.vehicle_status_sub.destroy_node()
        self.route_publisher.destroy_node()
        self.executor.shutdown()

        