import time
import numpy as np
import kdtree
import yaml

import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

class Seed:
    def __init__(self, color, xy_position, seed_id=None): 
        self.color = color

        self.pose = Pose()
        self.pose.position.x = xy_position[0]
        self.pose.position.y = xy_position[1]
        self.pose.position.z = 0.1

        self.seed_id = seed_id
        self.picked = False

    def setPosition(self, x, y, z): 
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z

    def getName(self): 
        return "seed_" + self.color + "_" + str(self.seed_id)

    def getXYPosition(self):
        return (self.pose.position.x, self.pose.position.y)

class SeedManager:
    max_seed_per_color = 1000

    spawned_seed_ids = {}
    spawned_seed_ids["red"] = np.zeros(max_seed_per_color, dtype="int")
    spawned_seed_ids["blue"] = np.zeros(max_seed_per_color, dtype="int")
    spawned_seed_ids["black"] = np.zeros(max_seed_per_color, dtype="int")

    seed_xy_database = {}
    seed_tree = kdtree.create(dimensions=2)

    spawn_service_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_service_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    def spawn(seed): 
        if (seed.seed_id == None):
            assert(len(np.where(SeedManager.spawned_seed_ids[seed.color]==0)[0]) != 0)
            # set new spawned seed id with smallest unused id 
            seed.seed_id = np.where(SeedManager.spawned_seed_ids[seed.color]==0)[0][0]

        model_name = seed.getName() 
        model_path = rospkg.RosPack().get_path("planning_final_project") + "/models/"

        if (seed.color == "red"):  
            model_path += "seed_red/model.sdf"

        elif (seed.color == "blue"):  
            model_path += "seed_blue/model.sdf"

        elif (seed.color == "black"):  
            model_path += "seed_black/model.sdf"

        model_sdf = open(model_path, 'r').read()

        SeedManager.spawn_service_client(model_name, model_sdf, "/", seed.pose, "world")
        SeedManager.spawned_seed_ids[seed.color][seed.seed_id] = 1
        SeedManager.seed_xy_database[seed.getXYPosition()] = seed
        SeedManager.seed_tree.add(seed.getXYPosition())

    def delete(seed):
        model_name = seed.getName()
        SeedManager.delete_service_client(model_name)
        SeedManager.spawned_seed_ids[seed.color][seed.seed_id] = 0
        SeedManager.seed_tree.remove(seed.getXYPosition())

    def spawnForestSeeds(forest): 
        seeds_per_tree = 10
        colors = ["red", "blue", "black"]
        color_distribution = [0.5, 0.3, 0.2]
        for tree_xy in forest: 
            mean = tree_xy
            cov = np.diag(np.full(2, 0.2))
            seeds_xy = np.random.multivariate_normal(mean, cov, seeds_per_tree)
            random_color = np.random.choice(colors, 1, p=color_distribution)[0]

            for seed_xy in seeds_xy: 
                if (np.linalg.norm(np.array(tree_xy) - seed_xy) < 0.35):
                    continue

                seed = Seed(random_color, seed_xy)
                SeedManager.spawn(seed)
                rospy.sleep(0.1)

        rospy.loginfo("Finish spawning seeds for forest.")

    def getClosestSeed(xy_position, dist=0.05): 
        nn_seed_info = SeedManager.seed_tree.search_nn(xy_position)
        if (nn_seed_info == None): 
            rospy.loginfo("No seeds on map.")
            return None

        seed_xy = nn_seed_info[0].data
        seed_range = nn_seed_info[1]

        if (seed_range > dist):
            rospy.loginfo("No seeds near turtlebot.")
            return None

        seed = SeedManager.seed_xy_database[seed_xy]

        return seed

    def collectSeed(xy_position, dist=0.05): 
        seed = SeedManager.getClosestSeed(xy_position, dist)
        if (seed == None): 
            return None

        seed_xy = seed.getXYPosition()

        seed = SeedManager.seed_xy_database[seed_xy]
        rospy.loginfo("{} seed has been collected at {}.".format(seed.color, seed_xy))
        seed.picked = True 
        SeedManager.delete(seed)
        SeedManager.seed_tree.remove(seed_xy)

        return seed

    def plantSeed(seed): 
        rospy.loginfo("{} seed has been planted at {}.".format(seed.color, seed.getXYPosition()))
        SeedManager.spawn(seed)

    def dumpSeedsPosition(file_name="seeds100.yaml"): 
        file_path = rospkg.RosPack().get_path("planning_final_project") + "/maps/" + file_name
        seeds_list = []
        for seed_xy, seed in SeedManager.seed_xy_database.items(): 
            seeds_list.append([str(seed.color), \
                               (float(seed_xy[0]), float(seed_xy[1])), \
                               int(seed.seed_id)])

        seeds_data = {"seeds": seeds_list}

        with open(file_path, 'w') as f:
            data = yaml.dump(seeds_data, f)

    def loadSeeds(file_name="seeds100.yaml"): 
        file_path = rospkg.RosPack().get_path("planning_final_project") + "/maps/" + file_name

        with open(file_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        for seed_list in data["seeds"]: 
            seed_color = seed_list[0]
            seed_xy = seed_list[1]
            seed_id = seed_list[2]
            seed = Seed(seed_color, seed_xy, seed_id)
            SeedManager.spawned_seed_ids[seed_color][seed_id] = 1
            SeedManager.seed_xy_database[seed_xy] = seed
            SeedManager.seed_tree.add(seed_xy)


if __name__ == "__main__": 
    rospy.init_node("seed_node")

    # forest = [(0, 0)]
    # SeedManager.spawnForestSeeds(forest)
    # seed1 = Seed("red", (3, 3))
    # SeedManager.spawn(seed1)
    # seed2 = Seed("red", (0.1, 0.1))
    # SeedManager.spawn(seed2)
    # seed3 = Seed("red", (-0.1, -0.1))
    # SeedManager.spawn(seed3)
    # seed4 = Seed("red", (0.1, -0.1))
    # SeedManager.spawn(seed4)
    #
    # time.sleep(2)
    # SeedManager.collectSeed((0.2, 0.2))
    # time.sleep(2)
    # SeedManager.collectSeed((0.2, 0.2))
    # SeedManager.plantSeed(seed2)
    SeedManager.loadSeeds()
