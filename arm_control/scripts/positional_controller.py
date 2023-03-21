#!/usr/bin/python
import movement_commands
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose

class PositionalController():
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

    def initSubscribers(self):
        rospy.Subscriber("robot/positions", Pose, self.callback)
        rospy.Subscriber('systemManager/isReady', Bool, self.callbackSystemManager)
        return

    def initPublishers(self):
        self.pub = rospy.Publisher('robot/isMoving', String, queue_size=10)
        self.pubReady = rospy.Publisher('robot/isReady', Bool, queue_size=10)
        return
    
    def initVariables(self):
        self.stopPubReady = False
        self.move = movement_commands.ur10_move()    
        self.rate = rospy.Rate(50)
        return
    
    def callbackSystemManager(self, msg):
        self.stopPubReady = msg.data
        return
    
    def callback(self, msg):
        rospy.loginfo("[%s] Position control request is received!", self.name)
        rospy.loginfo("[%s] - x: %s, y: %s, z: %s" , self.name, msg.position.x, msg.position.y, msg.position.z)
        rospy.loginfo("[%s] - theta4_offset: %s, theta5_offset: %s, theta6_offset: %s, isLinear: %s" , self.name, msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        rospy.loginfo("[%s] Robot will move now ...", self.name)
        self.pub.publish("True")
        is_moved = self.move.moveRobotToPos(msg.position.x,msg.position.y,msg.position.z,msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
        rospy.loginfo("[%s] Robot will stop now ...", self.name)
        self.pub.publish(str( not is_moved))
        return
    
    def main(self):
        rospy.loginfo("[%s] Node configuration OK", self.name)
        while not rospy.is_shutdown():
            if not self.stopPubReady:
                self.pubReady.publish(True)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        name = 'position_controller'
        PositionalController(name)
    except:
        pass
