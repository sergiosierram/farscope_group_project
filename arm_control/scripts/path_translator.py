#!/usr/bin/python
import rospy, time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sys import stdin

class PathTranslator(object):
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
        self.shelf_calibrated_topic = rospy.get_param("~shelf_calibrated_pos_topic", "/shelf_calibrated_pos")
        self.cabinet_topic = rospy.get_param("~cabinet_pos_topic", "/cabinet_pos")
        self.object_topic = rospy.get_param("~object_pos_topic", "/object_pos")
        self.translated_topic = rospy.get_param("~translated_topic", "/translated_pos")
        self.mux_rate = rospy.get_param("~rate", 10)
        return

    def initSubscribers(self):
        self.sub_shelf_calibrated = rospy.Subscriber(self.shelf_calibrated_topic, Pose, self.callbackShelfCalibratedPos)
        self.sub_cabinet = rospy.Subscriber(self.cabinet_topic, Pose, self.callbackCabinet)
        self.sub_object = rospy.Subscriber(self.object_topic, Pose, self.callbackOject)
        return

    def initPublishers(self):
        self.pub_translated = rospy.Publisher(self.translated_topic, Pose, queue_size = 10)
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        return


    def callbackShelfCalibratedPos(self, msg):
        return
    
    def callbackCabinet(self, msg):
        return

    def callbackOject(self, msg):
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ac = PathTranslator("PathTranslator")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
