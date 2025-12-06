#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class ArduinoEmulator:
    def __init__(self):
        rospy.init_node('arduino_emulator')
        
        # Subscriber to receive velocity commands
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        rospy.loginfo("Arduino emulator ready to receive velocity commands")

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands and simulate motor control"""
        
        rospy.loginfo(f"Received cmd_vel: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}")

if __name__ == '__main__':
    try:
        emulator = ArduinoEmulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
