import rclpy
import threading
from rclpy import executors
from std_msgs.msg import * 
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *

from drone_command.route_planner import RoutePlannerGUI
from drone_command.setpoint_control_gui import SetpointControlGUI

class DroneControlMenu(QWidget):
    
    # main menu layout
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Control")
        self.setFont(QFont("Arial", 16))
        self.setWindowIcon(QIcon("/home/gugafelds/lps_drone_sim/src/drone_command/images/lps_logo_mini.png"))
        self.setFixedSize(QSize(400, 400))
        self.setStyleSheet("background-color: white;")

        layoutMain = QFormLayout()
        self.setLayout(layoutMain)

        self.lps_logo_label = QLabel(self)
        self.lps_logo_img = QPixmap("/home/gugafelds/lps_drone_sim/src/drone_command/images/top_img.png")
        self.lps_logo_label.setPixmap(self.lps_logo_img)
        self.lps_logo_label.setScaledContents(True)

        self.title_label = QLabel(self)
        self.title_label.setText("Simulation of UAVs in Offboard Mode")
        self.title_label.setFont(QFont("Arial", 16, weight=QFont.Bold))
        self.title_label.setWordWrap(True)


        self.drone_control_gui_label = QLabel(self)
        self.drone_control_gui_label.setText("Drone Control:")
        self.drone_control_gui_label.setFont(QFont("Arial", 16, weight=QFont.Bold))

        self.setpoint_control_button = QPushButton("Setpoint Control")
        self.setpoint_control_button.clicked.connect(self.toggle_setpoint_control)

        self.route_planner_button = QPushButton("Route Planner")
        self.route_planner_button.clicked.connect(self.toggle_route_planner)

        self.exit_button = QPushButton("Exit")
        self.exit_button.clicked.connect(self.destroy_nodes)
        
        layoutMain.addRow(self.lps_logo_label)
        layoutMain.addRow(self.title_label)
        layoutMain.addRow(self.drone_control_gui_label)
        layoutMain.addRow(self.setpoint_control_button)
        layoutMain.addRow(self.route_planner_button)
        layoutMain.addRow(self.exit_button)

        self.subscriber_init()

    # starts setpoint and velocity forward option
    def toggle_setpoint_control(self):
        self.setpoint_control = SetpointControlGUI()
        self.setpoint_control.show()
        self.setEnabled(False)

    # starts route planner option
    def toggle_route_planner(self):
        self.route_planner = RoutePlannerGUI()
        self.route_planner.show()
        self.setEnabled(False)
    
    # callback function to control the gui
    def buttons_control(self, msg):
        self.setEnabled(msg.data)
    
    # subscribers on thread executor
    def subscriber_init(self):
        self.main_menu_sub = rclpy.create_node("main_menu_subscriber")
        self.main_menu_sub_ = self.main_menu_sub.create_subscription(Bool, "/com/main_menu", self.buttons_control, 10)

        self.main_menu_executor = executors.MultiThreadedExecutor()
        self.main_menu_executor.add_node(self.main_menu_sub)

        self.executor_thread = threading.Thread(target=self.main_menu_executor.spin, daemon=True)
        self.executor_thread.start()

    def destroy_nodes(self):
        self.main_menu_sub.destroy_node()
        self.main_menu_executor.shutdown()
        self.close()

