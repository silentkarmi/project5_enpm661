import time
import numpy as np
import cv2 as cv
import yaml

import rospy
import rospkg

from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

from seed import SeedManager

class Forest:

    start_points = (0, 0)
    x_max = 51
    y_max = 51
    end_points = (x_max, y_max)
    no_forest_radius = 25 # no trees will be grown within this radius from end_points
    tree_radius = 0.3

    forest_grid = np.zeros((y_max, x_max))
    GRID_NO_FOREST = 2
    GRID_TREE = 1

    total_trees = 0
    spawned_trees = set()

    spawn_service_client = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_service_client = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    def createForestGrid(): 
        y_max = Forest.y_max
        x_max = Forest.x_max
        r = Forest.no_forest_radius
        end_points = np.array(Forest.end_points)
        for y in range(y_max - r, y_max): 
            for x in range(x_max - r, x_max): 
                if (np.linalg.norm(end_points - (x, y)) < r):
                    Forest.forest_grid[y_max-y-1][x] = Forest.GRID_NO_FOREST

        # cv.imshow("Forest", Forest.forest_grid)
        # cv.waitKey(0)

    def spawnTree(amount=20): 
        model_path = rospkg.RosPack().get_path("mrs_gazebo_common_resources") + "/models/tree_simple/model.sdf"
        model_sdf = open(model_path, 'r').read()

        Forest.createForestGrid()

        initial_pose = Pose()
        initial_pose.position.z = -0.1

        while (Forest.total_trees < amount): 
            x = np.random.randint(0, Forest.x_max)
            y = np.random.randint(0, Forest.y_max)

            if (Forest.forest_grid[Forest.y_max-y-1][x] > 0):
                continue

            model_name = "tree_simple_" + str(Forest.total_trees)   

            initial_pose.position.x = x
            initial_pose.position.y = y

            Forest.spawn_service_client(model_name, model_sdf, "/", initial_pose, "world")
            Forest.forest_grid[Forest.y_max-y-1][x] = Forest.GRID_TREE
            Forest.total_trees += 1
            rospy.sleep(0.1)

    def deleteTrees(amount=20): 
        for i in range(0, amount): 
            model_name = "tree_simple_" + str(i)
            rospy.loginfo("delete ", model_name)
            Forest.delete_service_client(model_name)
            rospy.sleep(0.1)

    def dumpTreesPosition(file_name="forest50.yaml"): 
        file_path = rospkg.RosPack().get_path("planning_final_project") + "/maps/" + file_name

        trees_y, trees_x = np.where(Forest.forest_grid == Forest.GRID_TREE)
        trees_y = Forest.y_max - trees_y - 1
        trees_position = np.vstack((trees_x, trees_y)).T.tolist() 
        forest_data = {"start": Forest.start_points,
                       "end": Forest.end_points,
                       "tree_radius": Forest.tree_radius,
                       "trees_position": trees_position}

        with open(file_path, 'w') as f:
            data = yaml.dump(forest_data, f)

    def loadForest(file_name="forest50.yaml"): 
        file_path = rospkg.RosPack().get_path("planning_final_project") + "/maps/" + file_name

        with open(file_path) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)

        # model_path = rospkg.RosPack().get_path("mrs_gazebo_common_resources") + "/models/tree_simple/model.sdf"
        # model_sdf = open(model_path, 'r').read()

        Forest.createForestGrid()

        initial_pose = Pose()
        initial_pose.position.z = -0.1

        for tree in data["trees_position"]: 
            x, y = tree
            # model_name = "tree_simple_" + str(Forest.total_trees)   

            # initial_pose.position.x = x
            # initial_pose.position.y = y

            # Forest.spawn_service_client(model_name, model_sdf, "/", initial_pose, "world")
            Forest.forest_grid[Forest.y_max-y-1][x] = Forest.GRID_TREE
            Forest.total_trees += 1

        rospy.loginfo("Finish loading forest.")

        return data["trees_position"]

if __name__ == "__main__": 
    rospy.init_node("forest_node")
    # Forest.deleteTrees(50)
    # Forest.spawnTree(200)
    # Forest.dumpTreesPosition("forest200.yaml")
    forest = Forest.loadForest("forest200.yaml")
    SeedManager.spawnForestSeeds(forest)
    SeedManager.dumpSeedsPosition("seeds200.yaml")

