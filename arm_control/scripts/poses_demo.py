#!/usr/bin/python
import rospy, time, copy
import sys, moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Pose
import kinematics

class PosesDemo():
    def __init__(self, name):
        self.name = name
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('PosesDemo', anonymous = True)
        rospy.loginfo("[%s] Starting pose demonstrator", self.name)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.initVariables()
        self.mainControl()

    def initParameters(self):
        # Moveit stuff
        self.trajectoy_topic = rospy.get_param("~trajectoy_topic", "/move_group/display_planned_path")
        # Additional stuff
        self.rate_value = rospy.get_param("~node_parameters/rate",1)
        return

    def initSubscribers(self):
        rospy.loginfo("[%s] Initializing subscribers", self.name)
        rospy.Subscriber("shelf/selector", String, self.callbackShelf)
        return

    def initPublishers(self):
        rospy.loginfo("[%s] Initializing publishers", self.name)
        # Moveit stuff
        self.display_trajectory_publisher = rospy.Publisher(self.trajectoy_topic, DisplayTrajectory, queue_size = 20)
        self.ismoving_pub = rospy.Publisher('robot/isMoving', String, queue_size = 20)
        return

    def initVariables(self):
        # Moveit Vars
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        # Additional Vars
        self.joints = [
                        [5.414174556732178, -1.9323938528644007, -1.246256176625387, -0.01906282106508428, -0.5828827063189905, -3.0969911257373255],
                        [5.096587657928467, -1.8935135046588343, -1.2884400526629847, -0.01879913011659795, -0.35438663164247686, -3.094884697590963],
                        [4.806588649749756, -1.88788348833193, -1.2767422834979456, -0.03195697466005498, -0.1639636198626917, -3.095842425023214],
                        [5.365898132324219, -2.1077006498919886, -1.569625202809469, 0.2727639675140381, -0.522898022328512, -2.9565418402301233],
                        [5.115416526794434, -1.955104176198141, -1.7171972433673304, 0.2755308151245117, -0.31705934206117803, -2.956864658986227],
                        [4.801599502563477, -1.957726780568258, -1.7058971563922327, 0.275051474571228, -0.10432798067201787, -2.956852738057272],
                        [4.958446502685547, -2.379840675984518, -1.841811482106344, 0.38452887535095215, 0.08770018815994263, -2.4283233324633997],
                        [4.976724624633789, -2.753434960042135, -1.824141804371969, 0.21251404285430908, 0.003852179506793618, -1.8834951559649866]]
        self.rate = rospy.Rate(self.rate_value)
        return

    def basicInfo(self):
        rospy.loginfo("[%s] Basic Information of the Robot", self.name)
        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo("==> Planning frame: %s", planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("==> End effector link: %s", eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        rospy.loginfo("==> Available Planning Groups: %s", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the robot:
        rospy.loginfo("==> Printing robot state %s", self.robot.get_current_state())
        print("")
        return

    def planningToJoints(self, q0, q1, q2, q3, q4, q5):
        # We get the joint values from the group and change some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = q0
        joint_goal[1] = q1
        joint_goal[2] = q2
        joint_goal[3] = q3
        joint_goal[4] = q4
        joint_goal[5] = q5

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        return

    def planningToPose(self, w, x, y, z):
        pose_goal = Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        self.move_group.set_pose_target(pose_goal)

        print("Exec")
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()
        return

    def cartesianPath(self):
        waypoints = []
        scale = 1.0

        wpose = self.move_group.get_current_pose().pose
        print(wpose)
        return
        #wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.3  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.3  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.3  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def cartesianPath2(self):
        waypoints = []
        scale = 1.0

        wpose = self.move_group.get_current_pose().pose
        #wpose.position.z -= scale * 0.1  # First move up (z)

        wpose.position.z += scale * 0.3  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def setJointPosWait(self, theta0, theta1,  theta2, theta3, theta4, theta5):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = theta0
        joint_goal[1] = theta1
        joint_goal[2] = theta2
        joint_goal[3] = theta3
        joint_goal[4] = theta4
        joint_goal[5] = theta5
        
        is_at_pos = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        time.sleep(0.01)
        return is_at_pos

    def callbackShelf(self, msg):
        print("received "+msg.data)
        idx = int(msg.data)
        pose = self.joints[idx]
        print(pose)
        self.ismoving_pub.publish("true")
        result = self.setJointPosWait(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
        self.ismoving_pub.publish("false")
        print(result)
        return

    def mainControl(self):
        rospy.loginfo("[%s] Pose demonstrator OK", self.name)
        #self.planningToPose(1, 0.5, 0.5, 0.5 )
        #self.planningToPose(1, 0.5, -0.3, 1.0 )
        #time.sleep(3)
        #self.cartesianPath()
        #self.mainControl()
        #plan, fraction = self.cartesianPath()
        #self.move_group.execute(plan, wait=True)
        #time.sleep(15)
        #print("---------------------------------------------")
        #plan, fraction = self.cartesianPath2()
        #self.move_group.execute(plan, wait=True)
        while not rospy.is_shutdown():
        #    """ aux = self.joints[count]
        #    rospy.loginfo("[%s] Sending pose [%d]", self.name, count)
        #    self.planningToPose(aux[0], aux[1], aux[2], aux[3])
        #    count += 1
        #    if count == len(self.joints):
        #        break """
        #    self.basicInfo()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = PosesDemo('poses_demo')
    except rospy.ROSInterruptException:
        pass
