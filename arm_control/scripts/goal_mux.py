#!/usr/bin/python
import rospy, time
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from sys import stdin

class GoalMultiplexer(object):
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
        self.home_topic = rospy.get_param("~home_topic", "/home_pos")
        self.home_priority = rospy.get_param("~home_priority", 100)
        self.bin_topic = rospy.get_param("~bin_topic", "/bin_pos")
        self.bin_priority = rospy.get_param("~bin_priority", 10)
        self.translated_topic = rospy.get_param("~translated_topic", "/translated_pos")
        self.translated_priority = rospy.get_param("~translated_priority", 50)
        self.emergency_topic = rospy.get_param("~emergency_topic", "/emergency")
        self.emergency_delay = rospy.get_param("~emergency_topic", 10)
        self.pause_topic = rospy.get_param("~pause_topic", "/pause")
        self.pause_delay = rospy.get_param("~pause_delay", 1)
        self.goal_topic = rospy.get_param("~goal_topic", "/goal")
        self.mux_rate = rospy.get_param("~rate", 10)
        self.timeout = rospy.get_param("~timeout", 1)
        return

    def initSubscribers(self):
        self.sub_home = rospy.Subscriber(self.home_topic, Pose, self.callbackHome)
        self.sub_bin = rospy.Subscriber(self.bin_topic, Pose, self.callbackBin)
        self.sub_translated = rospy.Subscriber(self.translated_topic, Pose, self.callbackTranslated)
        self.sub_emergency = rospy.Subscriber(self.emergency_topic, Bool, self.callbackEmergency)
        self.sub_pause = rospy.Subscriber(self.pause_topic, Bool, self.callbackPause)
        return

    def initPublishers(self):
        self.pub_goal = rospy.Publisher(self.goal_topic, Pose, queue_size = 10)
        return

    def initVariables(self):
        self.rate = rospy.Rate(self.mux_rate)
        self.emergency = False
        self.pause = False
        self.msg_pose = Pose()
        self.changeHome = False
        self.changeBin = False
        self.changeTranslated = False
        self.priorities = [ [self.home_priority, self.home_topic, None, time.time()],
                            [self.bin_priority, self.bin_topic, None, time.time()],
                            [self.translated_priority, self.translated_topic, None, time.time()] ]
        self.t1 = 0
        self.t0 = 0
        self.dt = 10
        return


    def callbackHome(self, msg):
        if self.home_priority > 0:
            self.priorities[0][2] = msg
            self.priorities[0][3] = time.time()
            self.changeHome = True
            
        return
    
    def callbackBin(self, msg):
        if self.bin_priority > 0:
            self.priorities[1][2] = msg
            self.priorities[1][3] = time.time()
            self.changeBin = True
        return

    def callbackTranslated(self, msg):
        if self.translated_priority > 0:
            self.priorities[2][2] = msg
            self.priorities[2][3] = time.time()
            self.changeTranslated = True
        return

    def callbackEmergency(self, msg):
        self.emergency = msg.data
        return

    def callbackPause(self, msg):
        self.pause = msg.data
        return

    def main(self):
        rospy.loginfo("[%s] Configuration OK", self.name)
        while not rospy.is_shutdown():
            if not self.emergency:
                if not self.pause:
                    self.t1 = time.time()
                    self.dt = self.t1 - self.t0
                    if (self.dt > self.timeout) and (self.changeHome or self.changeBin or self.changeTranslated):
                        # Only when a goal is received for at least one topic, we proceed.
                        # The topics are sorted according to the priorities in descending order
                        aux = sorted(self.priorities, key=lambda x: x[0], reverse=True)
                        if self.changeHome and aux[0][2] != None:
                            pose = aux[0][2]
                            self.changeHome = False
                        elif self.changeBin and aux[1][2] != None:
                            pose = aux[1][2]
                            self.changeBin = False
                        elif self.changeTranslated and aux[2][2] != None:
                            pose = aux[2][2]
                            self.changeTranslated = False
                        else: 
                            continue
                        self.pub_goal.publish(pose)
                        self.priorities[0][2] = None
                        self.priorities[1][2] = None
                        self.priorities[2][2] = None
                else:
                    # On pause, incoming messages are ignored
                    rospy.loginfo("[%s] Node is paused, waiting for %d seconds", self.name, self.pause_delay)
                    time.sleep(self.pause_delay)
            else:
                # On emergency, 0s are passed in all fields of the Pose msg
                rospy.logwarn("[%s] Emergency is called, waiting for %d seconds", self.name, self.emergency_delay)
                time.sleep(self.emergency_delay)
                self.msg_pose = Pose()
                self.msg_pose.position.x = 0
                self.msg_pose.position.y = 0
                self.msg_pose.position.z = 0
                self.msg_pose.orientation.x = 0
                self.msg_pose.orientation.y = 0
                self.msg_pose.orientation.z = 0
                self.msg_pose.orientation.w = 0
                self.pub_goal.publish(self.msg_pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        ac = GoalMultiplexer("GoalMultiplexer")
    except rospy.ROSInterruptException:
        pass
        print('Exiting Admitance Controller')
