#!/usr/bin/env python


# Python 2/3 compatibility imports
from __future__ import print_function
#from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ###
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "kuka_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        #print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.cylinder_name=""
        self.robot = robot
        #self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    

    def go_to_joint_state(self, pos):
    
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        if pos=="home":
            #home
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-72)
            joint_goal[2] = np.deg2rad(62)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-99)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()
        if pos=="center":
            #New point 1(filtersentrert)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-73)
            joint_goal[2] = np.deg2rad(71)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-91)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()
        if pos=="filter1" or pos=="filter2drop":
            #New point 2(filterpåveined2
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-71)
            joint_goal[2] = np.deg2rad(82)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-78)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

            #New point 3filterpåveined3)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-65)
            joint_goal[2] = np.deg2rad(90)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-65)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

            # New point 4(filterpåveined4)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-54)
            joint_goal[2] = np.deg2rad(95)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-49)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()
            #FIlterbottom
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-51)
            joint_goal[2] = np.deg2rad(95)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-45)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

        elif pos=="filter1direkte":
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-51)
            joint_goal[2] = np.deg2rad(95)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-45)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

        elif pos=="filter1up":

            # New point 4(filterpåveined4)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-54)
            joint_goal[2] = np.deg2rad(95)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-49)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

            #New point 3filterpåveined3)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-65)
            joint_goal[2] = np.deg2rad(90)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-65)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

            #New point 2(filterpåveined2
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-71)
            joint_goal[2] = np.deg2rad(82)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-78)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

            #New point 1(filtersentrert)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-73)
            joint_goal[2] = np.deg2rad(71)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-91)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

            #home
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = np.deg2rad(-4)
            joint_goal[1] = np.deg2rad(-72)
            joint_goal[2] = np.deg2rad(62)
            joint_goal[3] = np.deg2rad(180)
            joint_goal[4] = np.deg2rad(-99)
            joint_goal[5] = np.deg2rad(-68)
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()

           
        elif pos=="filter1predrop":
            #Third position 
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] =-0.83775804096
            joint_goal[1] = -1.308996939
            joint_goal[2] = 1.1344640138
            joint_goal[3] = 3.14159
            joint_goal[4] = -1.710422667
            joint_goal[5] = -1.169370
            #Rad: J1: -0.83775804096, J2: -1.308996939, J3: 1.1344640138, J4:3.14159,J5: -1.710422667, J6:-1.169370

            move_group.go(joint_goal, wait=True)
            move_group.stop()
        elif pos=="filter1drop":
            #Fourth position
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.83775804096
            joint_goal[1] = -0.89011791852
            joint_goal[2] = 1.6929693744
            joint_goal[3] = 3.14159
            joint_goal[4] = -0.76794487088
            joint_goal[5] = -1.169370
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            #self.detach_cylinder()
            #self.remove_cylinder()
            #Rad: J1: -0.83775804096, J2: -0.89011791852, J3:1.6929693744, J4:3.14159,J5: -0.76794487088, J6:-1.169370
        elif pos=="filter2":
            #Fifth positon
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.73303828584
            joint_goal[1] = -0.6108652382
            joint_goal[2] = 1.2566370614
            joint_goal[3] = 3.14159
            joint_goal[4] = -0.92502450356
            joint_goal[5] = -1.0646508437

            move_group.go(joint_goal, wait=True)
            move_group.stop()
            #self.add_cylinder()
            #self.attach_cylinder()
            #Rad: J1: -0.73303828584, J2: -0.6108652382, J3: 1.2566370614, J4:3.14159,J5: -0.92502450356, J6-1.0646508437

        elif pos=="filter2up":
            #sixth position
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.73303828584
            joint_goal[1] = -0.83775804096
            joint_goal[2] = 0.36651914292
            joint_goal[3] = 3.14159
            joint_goal[4] = -2.0420352248
            joint_goal[5] = -1.0646508437

            #Rad: J1: -0.73303828584, J2: -0.83775804096, J3: 0.36651914292, J4:3.14159,J5: -2.0420352248, J6: -1.0646508437

            move_group.go(joint_goal, wait=True)
            move_group.stop()

        elif pos=="filter2predrop":
            #seventh position(same as second)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.0698131
            joint_goal[1] = -1.3089969
            joint_goal[2] = 1.134464
            joint_goal[3] = 3.14159
            joint_goal[4] = -1.710422667
            joint_goal[5] = -1.169370
            #Rad: J1: -0.0698131, J2:-1.3089969, J3:1.134464, J4:3.14159,J5:-1.710422667, J6:-1.169370
            move_group.go(joint_goal, wait=True)
            move_group.stop()

        #elif pos=="filter2drop":
            #eight positon(same as first)
            #joint_goal = move_group.get_current_joint_values()
            #joint_goal = move_group.get_current_joint_values()
            #joint_goal[0] = -0.0698131
            #joint_goal[1] = -0.972664
            #joint_goal[2] = 1.67551
            #joint_goal[3] = 3.14159
            #joint_goal[4] = -0.715584
            #joint_goal[5] = -1.169370

            #move_group.go(joint_goal, wait=True)
            #move_group.stop()
            #self.detach_cylinder()
            #self.remove_cylinder()


        #elif pos=="home":
            #return to home positon(same as second)
            #joint_goal = move_group.get_current_joint_values()
            #joint_goal[0] = -0.0698131
            #joint_goal[1] = -1.3089969
            #joint_goal[2] = 1.134464
            #joint_goal[3] = 3.14159
            #joint_goal[4] = -1.710422667
            #joint_goal[5] = -1.169370
            #Rad: J1: -0.0698131, J2:-1.3089969, J3:1.134464, J4:3.14159,J5:-1.710422667, J6:-1.169370
            #move_group.go(joint_goal, wait=True)
            #move_group.stop()

        elif pos=="auto":
            
            #first position
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.06981317008
            joint_goal[1] = -0.82030474844
            joint_goal[2] = 1.6057029118
            joint_goal[3] = 3.1590459461
            joint_goal[4] = -0.80285145592
            joint_goal[5] = -1.1519173063
           
            move_group.go(joint_goal, wait=True)

            move_group.stop()
            #self.add_cylinder()
            #self.attach_cylinder()
       
            #move_group.execute(joint_goal, wait=True)
        
            #Second position
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.0698131
            joint_goal[1] = -1.3089969
            joint_goal[2] = 1.134464
            joint_goal[3] = 3.14159
            joint_goal[4] = -1.710422667
            joint_goal[5] = -1.169370
            #Rad: J1: -0.0698131, J2:-1.3089969, J3:1.134464, J4:3.14159,J5:-1.710422667, J6:-1.169370
            move_group.go(joint_goal, wait=True)
            move_group.stop()
        
            #Third position 
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] =-0.83775804096
            joint_goal[1] = -1.308996939
            joint_goal[2] = 1.1344640138
            joint_goal[3] = 3.14159
            joint_goal[4] = -1.710422667
            joint_goal[5] = -1.169370
            #Rad: J1: -0.83775804096, J2: -1.308996939, J3: 1.1344640138, J4:3.14159,J5: -1.710422667, J6:-1.169370

            move_group.go(joint_goal, wait=True)
            move_group.stop()
        
            #Fourth position
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.83775804096
            joint_goal[1] = -0.89011791852
            joint_goal[2] = 1.6929693744
            joint_goal[3] = 3.14159
            joint_goal[4] = -0.76794487088
            joint_goal[5] = -1.169370
            move_group.go(joint_goal, wait=True)
            move_group.stop()
            #self.detach_cylinder()
            #self.remove_cylinder()
            #Rad: J1: -0.83775804096, J2: -0.89011791852, J3:1.6929693744, J4:3.14159,J5: -0.76794487088, J6:-1.169370
        
            #Fifth positon
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.73303828584
            joint_goal[1] = -0.6108652382
            joint_goal[2] = 1.2566370614
            joint_goal[3] = 3.14159
            joint_goal[4] = -0.92502450356
            joint_goal[5] = -1.0646508437

            move_group.go(joint_goal, wait=True)
            move_group.stop()
           # self.add_cylinder()
            #self.attach_cylinder()
            #Rad: J1: -0.73303828584, J2: -0.6108652382, J3: 1.2566370614, J4:3.14159,J5: -0.92502450356, J6-1.0646508437

       
            #sixth position
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.73303828584
            joint_goal[1] = -0.83775804096
            joint_goal[2] = 0.36651914292
            joint_goal[3] = 3.14159
            joint_goal[4] = -2.0420352248
            joint_goal[5] = -1.0646508437

            #Rad: J1: -0.73303828584, J2: -0.83775804096, J3: 0.36651914292, J4:3.14159,J5: -2.0420352248, J6: -1.0646508437

            move_group.go(joint_goal, wait=True)
            move_group.stop()

   
            #seventh position(same as second)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.0698131
            joint_goal[1] = -1.3089969
            joint_goal[2] = 1.134464
            joint_goal[3] = 3.14159
            joint_goal[4] = -1.710422667
            joint_goal[5] = -1.169370
            #Rad: J1: -0.0698131, J2:-1.3089969, J3:1.134464, J4:3.14159,J5:-1.710422667, J6:-1.169370
            move_group.go(joint_goal, wait=True)
            move_group.stop()

        
            #eight positon(same as first)
            joint_goal = move_group.get_current_joint_values()
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.06981317008
            joint_goal[1] = -0.82030474844
            joint_goal[2] = 1.6057029118
            joint_goal[3] = 3.1590459461
            joint_goal[4] = -0.80285145592
            joint_goal[5] = -1.1519173063

            move_group.go(joint_goal, wait=True)
            move_group.stop()
            #self.detach_cylinder()
            #self.remove_cylinder()
        
            #return to home positon(same as second)
            joint_goal = move_group.get_current_joint_values()
            joint_goal[0] = -0.0698131
            joint_goal[1] = -1.3089969
            joint_goal[2] = 1.134464
            joint_goal[3] = 3.14159
            joint_goal[4] = -1.710422667
            joint_goal[5] = -1.169370
            #Rad: J1: -0.0698131, J2:-1.3089969, J3:1.134464, J4:3.14159,J5:-1.710422667, J6:-1.169370
            move_group.go(joint_goal, wait=True)
            move_group.stop()
        else:
            joint_goal = move_group.get_current_joint_values()
            print("Invalid inputs\n")
            print("Valid inputs are: filter1,filter1up,filter")
            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            move_group.go(joint_goal, wait=True)

            # Calling ``stop()`` ensures that there is no residual movement
            move_group.stop()

            ## END_SUB_TUTORIAL

            # For testing:
            current_joints = move_group.get_current_joint_values()
            return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group


        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = 0.554127
        pose_goal.orientation.y = 0
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0.832433 
        pose_goal.position.x = 1.1302
        pose_goal.position.y = -0.0837862
        pose_goal.position.z = 0.685435
        move_group.set_pose_target(pose_goal)
        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)
     
    

        ## END_SUB_TUTORIAL

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        
    # SJEKKER OM BOX ER FESTET PAA GRIPPER
    def wait_for_state_update(
        self, cylinder_is_known=False, cylinder_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([cylinder_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = cylinder_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (cylinder_is_attached == is_attached) and (cylinder_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        


    def add_cylinder(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = "link_6"
        cylinder_pose.pose.orientation.w = 0.7071
        cylinder_pose.pose.orientation.x = 0
        cylinder_pose.pose.orientation.y = 0.7071
        cylinder_pose.pose.position.z = 0 
        cylinder_pose.pose.position.y=-0
        cylinder_pose.pose.position.x=0.4
        cylinder_name = "cylinder"
        scene.add_cylinder(cylinder_name, cylinder_pose, height =0.3,radius=0.045)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.cylinder_name = cylinder_name
        return self.wait_for_state_update(cylinder_is_known=True, timeout=timeout)

    def attach_cylinder(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        scene = self.scene
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, cylinder_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            cylinder_is_attached=True, cylinder_is_known=False, timeout=timeout)

   

    def detach_cylinder(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=cylinder_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            cylinder_is_known=True, cylinder_is_attached=False, timeout=timeout
        )

    def remove_cylinder(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        cylinder_name = self.cylinder_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(cylinder_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            cylinder_is_attached=False, cylinder_is_known=False, timeout=timeout
        )
       


def main():
    try:
        """print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )"""
        tutorial = MoveGroupPythonInterfaceTutorial()
        while True:
            comment = input(
                "============ Press `Enter` to execute a movement using a joint state goal ..."
            )
            if comment == "break":
                break
        
            tutorial.go_to_joint_state(comment)
           
        
        """
        input("============ Press `Enter` to execute a movement using a pose goal ...")
        tutorial.go_to_pose_goal()
        
        input("============ Press `Enter` to add a box to the planning scene ...")
        tutorial.add_cylinder()
        input("============ Press `Enter` to plan and display a Cartesian path ...")
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan)

        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)"""

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()