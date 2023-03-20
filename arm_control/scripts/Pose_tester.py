import rospy
from geometry_msgs.msg import Pose

pub = rospy.Publisher('robot/positions', Pose)
rospy.init_node('test_pos', anonymous=True)

shelf_pose = Pose()
shelf_pose.position.x = -200
shelf_pose.position.y = 400
shelf_pose.position.z = 600
shelf_pose.orientation.x = 1.57
shelf_pose.orientation.y = 0
shelf_pose.orientation.z = 3.14
shelf_pose.orientation.w = 0

pub.publish(shelf_pose)
