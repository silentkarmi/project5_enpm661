import numpy as np
import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

class Seed:
    def __init__(self, color, position, seed_id=None): 
        self.color = color

        self.pose = Pose()
        self.pose.position.x = position[0]
        self.pose.position.y = position[1]
        self.pose.position.z = position[2]

        self.seed_id = seed_id
        self.name = "seed_" + color + "_" + str(seed_id)

    def setPosition(self, x, y, z): 
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z

class SeedSpawner:
    max_seed_per_color = 10

    spawned_seed_ids = {}
    spawned_seed_ids["red"] = np.zeros(max_seed_per_color, dtype="int")
    spawned_seed_ids["blue"] = np.zeros(max_seed_per_color, dtype="int")
    spawned_seed_ids["black"] = np.zeros(max_seed_per_color, dtype="int")

    spawn_service_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete_service_client = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def spawn(self, seed): 
        if (seed.seed_id == None):
            assert(len(np.where(Seed.spawned_seed_ids[seed.color]==0)[0]) != 0, "Seed exceed maximum amount")
            # set new spawned seed id with smallest unused id 
            seed.seed_id = np.where(Seed.spawned_seed_ids[seed.color]==0)[0][0]

        Seed.spawned_seed_ids.add(seed.seed_id)
        model_name = seed.name 
        model_path = rospkg.RosPack().get_path('planning_final_project') + "/models/"

        if (seed.color == red):  
            model_path += "seed_red/model.sdf"

        elif (seed.color == blue):  
            model_path += "seed_blue/model.sdf"

        elif (seed.color == black):  
            model_path += "seed_black/model.sdf"

        Seed.spawn_model_client(model_name, model_path, seed.pose, "world")

    def delete(self, seed):
        model_name = seed.name
        Seed.delete_service_client(model_name)


if __name__ == "__main__": 
    rospy.init_node("seed_node")

    seed1 = Seed("blue")
    seed1.setPosition(0, 0, 0.1)
    SeedSpawner.spawn(seed1)

    print(SeedSpawner.spawned_seed_ids["blue"])
