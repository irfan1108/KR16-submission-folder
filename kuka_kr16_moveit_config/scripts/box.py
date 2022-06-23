import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
rospy.init_node("scene_test",anonymous=True)

scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

rospy.sleep(2)

p = PoseStamped()
p.pose.position.x = 4
p.pose.position.y = 0.2
p.pose.position.z = 0.3

scene.add_box("table",p,(0.5,1.5,0.6))

