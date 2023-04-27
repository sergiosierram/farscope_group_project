#!/usr/bin/python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool

class ShelfSelect():
    def __init__(self, name):
        self.name = name
        rospy.init_node(self.name, anonymous=True)
        rospy.loginfo("[%s] Starting node", self.name)
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.main()

    def initSubscribers(self):
        rospy.Subscriber("shelf/selector", String, self.callback_shelf)
        rospy.Subscriber("systemManager/isReady", Bool, self.callbackSystemManager)
        return

    def initPublishers(self):
        self.pub = rospy.Publisher('robot/positions', Pose, queue_size = 10)
        self.pubReady = rospy.Publisher('shelf/isReady', Bool, queue_size=10)
        return
    
    def initVariables(self):
        self.stopPubReady = False
        self.shelf_corner_x = 71
        self.shelf_corner_z = 74
        self.shelf_z_offset = 319
        self.z_camera_offset = 40

        self.shelf_x_offset = 270
        self.shelf_y_offset = 590
        self.topCorner = (-450,1140,735)
        self.rate = rospy.Rate(50)
        return
    
    def callbackSystemManager(self, msg):
        self.stopPubReady = msg.data
        return
    
    def callback_shelf(self, msg):
        rospy.loginfo("[%s] Shelf request is received!", self.name)
        print(msg)
        inmsg = msg.data

        if inmsg.isdigit() and int(inmsg) <= 12 and int(inmsg) > 0:
            rospy.loginfo("[%s] Processing shelf selection with id: %d", self.name, int(inmsg))
            
            num = int(inmsg)
            x_offset = ((num-1) % 3)
            z_offset = int((num-1) / 3)

            x_pos_offset = self.shelf_corner_x + (self.shelf_x_offset * x_offset)
            z_pos_offset = self.shelf_corner_z + (self.shelf_z_offset * z_offset)

            x_pos = x_pos_offset + self.topCorner[0]
            z_pos = self.topCorner[2] - z_pos_offset - self.z_camera_offset

            shelf_pose = Pose()
            shelf_pose.position.x = x_pos
            shelf_pose.position.y = self.topCorner[1]- self.shelf_y_offset
            shelf_pose.position.z = z_pos
            shelf_pose.orientation.x = 1.57
            shelf_pose.orientation.y = 0
            shelf_pose.orientation.z = 3.14
            shelf_pose.orientation.w = 0

            rospy.loginfo("[%s] - x: %s, y: %s, z: %s" , self.name, shelf_pose.position.x, shelf_pose.position.y, shelf_pose.position.z)
            rospy.loginfo("[%s] - qx: %s, qy: %s, qz: %s, qw: %s" , self.name, shelf_pose.orientation.x, shelf_pose.orientation.y, shelf_pose.orientation.z, shelf_pose.orientation.w)

            self.pub.publish(shelf_pose)
            #print("Waiting for Shelf")

        else:
            rospy.logwarn("[%s] Skipping incorrect shelf index", self.name)
            #print("Waiting for Shelf")
        return

    def main(self):
        rospy.loginfo("[%s] Node configuration OK", self.name)
        while not rospy.is_shutdown():
            if not self.stopPubReady:
                self.pubReady.publish(True)
            self.rate.sleep()
    
if __name__ == '__main__':
    try:
        name = 'shelf_pos'
        ShelfSelect(name)
    except:
        pass

    
