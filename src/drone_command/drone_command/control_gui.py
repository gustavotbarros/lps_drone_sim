import rclpy

import sys

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from drone_command.main_menu import DroneControlMenu

class ControlGUI():
    def __init__(self):
        try:
            self.app = QApplication(sys.argv)
            self.main_menu = DroneControlMenu()
            self.main_menu.show()
            sys.exit(self.app.exec_())
        
        except RuntimeError as e:
            print(f"An error ocurred: {e}")
        finally:
            print("Closing GUI.")

def main(args=None):
    rclpy.init(args=args)
    control_gui = ControlGUI()
    rclpy.shutdown()

if __name__ == '__main__':
    main()