#!/usr/bin/python
import rospy, time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sys import stdin

class SystemManager(object):
    def __init__(self, name):
        self.name = name
        rospy.init_node(name, anonymous = True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

    def initParameters(self):
        self.machine_vision_topic = rospy.get_param("~machine_vision_topic", "/machine_vision")
        self.output_topic = rospy.get_param("~output_topic", "/output")
        self.home_topic = rospy.get_param("~home_topic", "/home_pos")
        self.pause_topic = rospy.get_param("~pause_topic", "/pause")
        self.emergency_topic = rospy.get_param("~emergency_topic", "/emergency")
        self.mux_rate = rospy.get_param("~rate", 10)
        return

    def initSubscribers(self):
        self.machine_vision_topic = rospy.Subscriber(self.machine_vision_topic, Pose, self.callbackMachineVision)
        return

    def initPublishers(self):
        self.pub_home = rospy.Publisher(self.home_topic, Pose, queue_size = 10)
        self.pub_output = rospy.Publisher(self.output_topic, Pose, queue_size = 10)
        self.pub_pause = rospy.Publisher(self.pause_topic, Bool, queue_size = 10)
        self.pub_emergency = rospy.Publisher(self.emergency_topic, Bool, queue_size = 10)
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        return


    def callbackMachineVision(self, msg):
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ac = SystemManager("SystemManager")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
