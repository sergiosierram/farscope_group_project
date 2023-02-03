#!/usr/bin/python3
import rospy
from std_msgs.msg import Bool

class GripperManager():
    def __init__(self, name):
        self.name = name
        rospy.init_node('Control', anonymous = True)
        rospy.loginfo("[%s] Starting gripper manager", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.mainControl()

    def initParameters(self):
        self.pump_topic = rospy.get_param("~pump_topic","pump_activate")
        self.rate_value = rospy.get_param("~node_parameters/rate",1)
        return

    def initSubscribers(self):
        rospy.loginfo("[%s] Initializing subscribers", self.name)
        return

    def initPublishers(self):
        rospy.loginfo("[%s] Initializing publishers", self.name)
        self.pub_pump = rospy.Publisher(self.pump_topic, Bool, queue_size=10)
        return

    def initVariables(self):
        self.count = 0
        self.state = True
        self.msg_pump = Bool()
        self.rate = rospy.Rate(self.rate_value)
        return

    def mainControl(self):
        rospy.loginfo("[%s] Gripper manager OK", self.name)
        while not rospy.is_shutdown():
            self.msg_pump.data = self.state
            self.pub_pump.publish(self.msg_pump)
            if self.count == 10:
                self.state = False
            else:
                self.count += 1
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = GripperManager('gripper_manager')
    except rospy.ROSInterruptException:
        pass