#!/usr/bin/python3
import sys
import rospy
from std_msgs.msg import String

class LEDController:
    def __init__(self):
        self._init_status_msgs()
        self._init_subscribers()
        self._init_publishers()
        self.status = 'FREE'

    def _init_subscribers(self):
        status_topic = rospy.get_param('~status_topic','status')
        if not status_topic:
            rospy.logerr('Parameter \'status_topic\' is not provided.')
            sys.exit(-1)
        
        rospy.Subscriber(status_topic, String, self._callback, queue_size=1)


    def _callback(self, msg):
        self.status = self.status_msg_dict.get(int(msg.data[0]))
        self.status_publisher.publish(self.status)


    def _init_status_msgs(self):
        self.status_msg_dict = {
                1 : 'FREE',
                2 : 'BUSY',
                3 : 'ARRIVED'
            }

    def _init_publishers(self):
        self.status_publisher = rospy.Publisher('~status', String, queue_size=1)


def main():
    rospy.init_node('led_controller', log_level=rospy.INFO)
    controller = LEDController()
    rospy.spin()


if __name__ == "__main__":
    main()