#!/usr/bin/python
import rospy, time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String
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
        rospy.Subscriber(self.machine_vision_topic, Pose, self.callbackMachineVision)
        rospy.Subscriber("robot/isMoving", String, self.callbackMoving)
        rospy.Subscriber("robot/positions", Pose, self.callbackPosition)
        return

    def initPublishers(self):
        self.pub_home = rospy.Publisher(self.home_topic, Pose, queue_size = 10)
        self.pub_output = rospy.Publisher(self.output_topic, Pose, queue_size = 10)
        self.pub_pause = rospy.Publisher(self.pause_topic, Bool, queue_size = 10)
        self.pub_emergency = rospy.Publisher(self.emergency_topic, Bool, queue_size = 10)
        self.pub_shelf = rospy.Publisher('shelf/selector', String, queue_size = 10)
        self.pub_pose  = rospy.Publisher('robot/positions', Pose, queue_size = 10)
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        self.isMoving = False
        self.currentPose = Pose()
        self.shelf_list = [0,1,2,3,4,5,6,7]
        return


    def callbackMachineVision(self, msg):
        return

    def callbackMoving(self, msg):
        print(msg.data)
        if(str(msg.data )== "False"):
            self.isMoving = False
            print("Done")
        else:
            print("isMoving")
            self.isMoving = True
        return

    def callbackPosition(self, pose):
        self.currentPose = pose
        return

    def shelfTalker(self, shelf_num):
        msg = String()
        msg.data = str(shelf_num)

        rospy.loginfo(msg)
        self.pub_shelf.publish(msg)
        return

    def positionTalker(self, Pose):
        self.pub_shelf.publish(Pose)
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            n = len(self.shelf_list)
            idx = 0
            while idx < n:
                if (self.isMoving == False):
                    self.shelfTalker(self.shelf_list[idx])
                    x=input("Next?")
                    time.sleep(0.01)
                else:
                    idx += 1
            user = input("Finished Shelf List. Want to go again?:").strip()
            if (user == "n" or user == "N"):
                break
            self.rate.sleep()
        return

if __name__ == '__main__':
    try:
        ac = SystemManager("SystemManager")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
