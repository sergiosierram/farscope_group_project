#!/usr/bin/python3
import rospy, time, copy
import sys, moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose

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
        return

    def initPublishers(self):
        rospy.loginfo("[%s] Initializing publishers", self.name)
        # Moveit stuff
        self.display_trajectory_publisher = rospy.Publisher(self.trajectoy_topic, DisplayTrajectory, queue_size = 20)
        return

    def initVariables(self):
        # Moveit Vars
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        # Additional Vars
        self.joints = [
                        [-1.2329629167348983, -0.15208868764668448, 0.2532445839785442, -0.17011216139049523, -1.605649292298132, 0.2799847649205507],
                        [-1.3800811928996275, -0.1528705045266774, -0.1825779630070894, -0.03674920934942527, -1.6041657061125445, -0.15603929999081156],
                        [-1.2247636568256333, -0.18211347003843592, -0.6376632056794413, -0.17542853615483978, -1.5964292498261727, -0.6111446261907423],
                        [-1.2329629167348983, -0.15208868764668448, 0.2532445839785442, -0.17011216139049523, -1.605649292298132, 0.2799847649205507],
                        [-1.3800811928996275, -0.1528705045266774, -0.1825779630070894, -0.03674920934942527, -1.6041657061125445, -0.15603929999081156],
                        [-1.2247636568256333, -0.18211347003843592, -0.6376632056794413, -0.17542853615483978, -1.5964292498261727, -0.6111446261907423],
                        [-1.2329629167348983, -0.15208868764668448, 0.2532445839785442, -0.17011216139049523, -1.605649292298132, 0.2799847649205507],
                        [-1.3800811928996275, -0.1528705045266774, -0.1825779630070894, -0.03674920934942527, -1.6041657061125445, -0.15603929999081156],
                        [-1.2247636568256333, -0.18211347003843592, -0.6376632056794413, -0.17542853615483978, -1.5964292498261727, -0.6111446261907423]]
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

    def mainControl(self):
        rospy.loginfo("[%s] Pose demonstrator OK", self.name)
        #self.planningToPose(1, 0.5, 0.5, 0.5 )
        #self.planningToPose(1, 0.5, -0.3, 1.0 )
        #time.sleep(3)
        plan, fraction = self.cartesianPath()
        self.move_group.execute(plan, wait=True)
        time.sleep(15)
        print("---------------------------------------------")
        plan, fraction = self.cartesianPath2()
        self.move_group.execute(plan, wait=True)
        #while not rospy.is_shutdown():
        #    """ aux = self.joints[count]
        #    rospy.loginfo("[%s] Sending pose [%d]", self.name, count)
        #    self.planningToPose(aux[0], aux[1], aux[2], aux[3])
        #    count += 1
        #    if count == len(self.joints):
        #        break """
        #    self.basicInfo()
        #    self.rate.sleep()

if __name__ == '__main__':
    try:
        node = PosesDemo('poses_demo')
    except rospy.ROSInterruptException:
        pass
