

#!/usr/bin/env pzthon
import rospy
from geometry_msgs.msg import Pose
import time
from std_msgs.msg import String

shelf_corner_x = 171
shelf_corner_z = 74
shelf_z_offset = 319
shelf_x_offset = 274.5
topCorner = (-195.5,800,860)

isMove = False

def callback_shelf(msg):
    print(msg)
    inmsg = msg.data

    if inmsg.isdigit() and int(inmsg) <= 12 and int(inmsg) > 0:
        print("correct")
        num = int(inmsg)
        x_offset = ((num-1) % 3)
        z_offset = int((num-1) / 3)

        x_pos_offset = shelf_corner_x + (shelf_x_offset * x_offset)
        z_pos_offset = shelf_corner_z + (shelf_z_offset * z_offset)

        x_pos = x_pos_offset + topCorner[0]
        z_pos = topCorner[2] - z_pos_offset

        shelf_pose = Pose()
        shelf_pose.position.x = x_pos
        shelf_pose.position.y = topCorner[1]
        shelf_pose.position.z = z_pos
        shelf_pose.orientation.x = 1.57
        shelf_pose.orientation.y = 0
        shelf_pose.orientation.z = 0
        shelf_pose.orientation.w = 0

        rospy.loginfo(shelf_pose)
        pub.publish(shelf_pose)
        print("Waiting for Shelf")

    else:
        print("incorrect shelf identifier")
        print("Waiting for Shelf")
    #isMove = msg.data

    
pub = rospy.Publisher('robot/positions', Pose)
rospy.init_node('shelf_pos', anonymous=True)
rospy.Subscriber("shelf/selector", String, callback_shelf)

def listener():
    print("Waiting for Shelf")
    while not rospy.is_shutdown():
        rospy.spin()
    

if __name__ == '__main__':
    listener()

    