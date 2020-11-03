#!/usr/bin/env python3

import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class Drone():
    def __init__(self):
        self.hz = 25
        self.rate = rospy.Rate(self.hz)
        self.state = State()
        self.receivedPosition = False
        rospy.wait_for_service("/mavros/cmd/arming")
        rospy.loginfo("/mavros/cmd/arming service ready!")
        rospy.wait_for_service("mavros/set_mode")
        rospy.loginfo("/mavros/set_mode service ready!")
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.setPoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)

        self.setup()

    def state_cb(self, state):
        self.state = state
        self.receivedPosition = True

    def setup(self):
        prevState = self.state


        self.takeOffPosition = PoseStamped()
        self.takeOffPosition.pose.position.x = 0
        self.takeOffPosition.pose.position.y = 0
        self.takeOffPosition.pose.position.z = 2

        print("Waiting for FCU connection...")
        while not self.state.connected:
            self.rate.sleep()
        print("FCU connected")

        print("Waiting on position...")
        while not self.receivedPosition:
            self.rate.sleep()
        print("Position received")

        # send a few takeoff commands before starting
        for i in range(20):
            self.setPoint_pub.publish(self.takeOffPosition)
            self.rate.sleep()

        print("Waiting for change mode to offboard & arming rotorcraft...")
        while not self.state.mode == "OFFBOARD" and not self.state.armed:
            if not self.state.mode == "OFFBOARD":
                self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                print("OFFBOARD enalbe")

            if not self.state.armed:
                self.arming_client(True)
                print("Rotorcraft armed")

            # send a few takeoff commands
            for i in range(20):
                self.setPoint_pub.publish(self.takeOffPosition)
                self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('drone_control', anonymous=True)
    drone = Drone()
    rospy.spin()
