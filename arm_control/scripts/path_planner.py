#!/usr/bin/python
import rospy, time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sys import stdin

class PathPlanner(object):
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
        self.goals_topic = rospy.get_param("~goals_topic", "/goals")
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.mux_rate = rospy.get_param("~rate", 10)
        return

    def initSubscribers(self):
        self.sub_goals = rospy.Subscriber(self.goals_topic, Pose, self.callbackGoals)
        self.sub_goal = rospy.Subscriber(self.goal_topic, Pose, self.callbackGoal)
        return

    def initPublishers(self):
        #self.pub_translated = rospy.Publisher(self.translated_topic, Pose, queue_size = 10)
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        return


    def callbackGoals(self, msg):
        return
    
    def callbackGoal(self, msg):
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ac = PathPlanner("PathPlanner")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
