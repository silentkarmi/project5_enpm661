import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

rospy.init_node("test_spawn")

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0.1 

rospy.wait_for_service('/gazebo/spawn_sdf_model')

spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
spawn_model_client("seed_blue_testing", open("/home/longhongc/model_editor_models/seed_blue/model.sdf", 'r').read(), "/", initial_pose, "world")
