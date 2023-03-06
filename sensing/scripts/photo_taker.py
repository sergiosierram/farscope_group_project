#!/usr/bin/python
import rospy, time, cv2, os
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

class PhotoTaker(object):
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
        self.photo_stream_topic = rospy.get_param("~photo_stream_topic", "/photo_stream")
        self.start_stream_topic = rospy.get_param("~start_stream_topic", "/start_stream")
        self.camera_port = rospy.get_param("~camera_port", 2)
        self.photo_taker_rate = rospy.get_param("~rate", 10)
        return

    def initSubscribers(self):
        self.sub_start = rospy.Subscriber(self.start_stream_topic, Bool, self.callbackStart)
        return

    def initPublishers(self):
        self.pub_photo = rospy.Publisher(self.photo_stream_topic, Image, queue_size = 10)
        return

    def initVariables(self):
        self.taking_photos = False
        self.camera = cv2.VideoCapture(self.camera_port)
        self.rate = rospy.Rate(self.photo_taker_rate)
        return

    def callbackStart(self, msg):
        self.taking_photos = msg.data
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            #Add photo taker code
            if self.taking_photos:
                print("Taking photos..")
                return_value, image = camera.read()
                msg = Image()
                msg.data = image
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ac = PhotoTaker("PhotoTaker")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
