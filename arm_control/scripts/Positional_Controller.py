#!/usr/bin/python
import movement_commands
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

move = movement_commands.ur10_move()

rospy.init_node('position_controller', anonymous=True)
pub = rospy.Publisher('robot/isMoving', String)

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg)
    print("x: " , msg.position.x)
    print("y: " , msg.position.y)
    print("z: " , msg.position.z)
    print("theta4_offset: " , msg.orientation.x)
    print("theta5_offset: " , msg.orientation.y)
    print("theta6_offset: " , msg.orientation.z)
    print("isLinear: " , msg.orientation.w)
    pub.publish("True")
    is_moved = move.moveRobotToPos(msg.position.x,msg.position.y,msg.position.z,msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w)
    pub.publish(str( not is_moved))

 
def listener():
    rospy.Subscriber("robot/positions", Pose, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
