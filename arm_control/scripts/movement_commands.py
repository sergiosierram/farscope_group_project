import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from include import kinematics
import time
import math


class ur10_move:
    def __init__(self):
        self.kin = kinematics.ur10_Kin()

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        self.theta5 = (0/180)*math.pi
        self.theta4_offset = (90/180)*math.pi
    
    def setJointPosWait(self, theta1,  theta2, theta3, theta4, theta5, theta6=0):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] =  theta1 - math.pi/2
        joint_goal[1] = -math.pi + theta2
        joint_goal[2] = -math.pi + theta3
        joint_goal[3] = -math.pi/2 + theta4
        joint_goal[4] = theta5
        joint_goal[5] = theta6
        
        is_at_pos = self.group.go(joint_goal, wait=True)
        self.group.stop()
        time.sleep(0.01)
        return is_at_pos
       
        

    def setJointPosNoWait(self, theta1,  theta2, theta3, theta4, theta5, theta6=0):
        joint_goal = self.group.get_current_joint_values()
        joint_goal[0] =  theta1 #- math.pi/2
        joint_goal[1] = -math.pi + theta2
        joint_goal[2] = -math.pi + theta3
        joint_goal[3] = -math.pi/2 + theta4
        joint_goal[4] = theta5
        joint_goal[5] = theta6

        self.group.go(joint_goal, wait=False)
        self.group.stop()

    def getRobotPos(self):
        x = self.kin.prevX
        y = self.kin.prevY
        z = self.kin.prevZ
        print("X: ", x, "Y: ", y, "Z: ", z)
        return x, y, z
    
    def moveRobotToPos(self, x, y, z, theta4off = 0, theta5 = 0, theta6 = 0, Linear = 0):
        if(Linear == 0):
            theta1, theta2, theta3, theta4, theta5_off = self.kin.solve_kinematics(x,y,z,theta5, theta4off)
            is_Moved = self.setJointPosWait(theta1, theta2, theta3, theta4, theta5 - theta5_off, theta6)
            return is_Moved
        else:
            print("LINEAR OHHHH")
            self.LinearMove(x,y,z, 20, 0.1)
            
            

    def LinearMove(self, x_end, y_end, z_end, steps, movedelay):
        x,y,z = self.getRobotPos()
        z_range = z - z_end
        y_range = y - y_end
        x_range = x - x_end

        theta1, theta2, theta3, theta4 = self.kin.solve_kinematics(x,y,z,self.theta5, self.theta4_offset)
        self.setJointPosWait(theta1, theta2, theta3, theta4, self.theta5)
        for i in range(steps-1):
            if(z_end > z):
                z_step = z + ((z_range/steps)*i)
            else:
                z_step = z - ((z_range/steps)*i)

            if(x_end > x):
                x_step = x + ((x_range/steps)*i)
            else:
                x_step = x - ((x_range/steps)*i)

            if(y_end > y):
                y_step = y + ((y_range/steps)*i)
            else:
                y_step = y - ((y_range/steps)*i)

            theta1, theta2, theta3, theta4 = self.kin.solve_kinematics(x_step, y_step, z_step,self.theta5, self.theta4_offset)
            self.setJointPosNoWait(theta1, theta2, theta3, theta4, self.theta5)
            time.sleep(movedelay)

        theta1, theta2, theta3, theta4 = self.kin.solve_kinematics(x_end,y_end,z_end,self.theta5, self.theta4_offset)
        self.setJointPosWait(theta1, theta2, theta3, theta4, self.theta5)

    def ScanShelf(self, shelfTop):

        theta1, theta2, theta3, theta4 = self.kin.solve_kinematics(shelfTop[0], shelfTop[1], shelfTop[2], self.theta5, self.theta4_offset)
        self.setJointPosWait(theta1, theta2, theta3, theta4, self.theta5)
        self.LinearMove(shelfTop[0],shelfTop[1], shelfTop[2] - 1200, 20, 0.2)
