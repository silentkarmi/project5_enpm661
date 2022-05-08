import numpy as np
import cv2 as cv

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from forest import Forest

if __name__ == "__main__": 
    rospy.init_node("map_create_node")

    empty_map = np.zeros((1200, 1200), dtype="uint8")

    trees_position = Forest.loadForest("forest100.yaml")
    for tree in trees_position:
        x, y = tree

        # scale to 1000
        x = (x + 10) * 20 
        y = (y + 10) * 20 

        cv.circle(empty_map, (x, y), 6, 100)

    empty_map_flat = empty_map.reshape(-1).tolist()


    occupancy_grid = OccupancyGrid()
    occupancy_grid.info.map_load_time = rospy.get_rostime()
    occupancy_grid.info.resolution = 0.05
    occupancy_grid.info.width = 1200
    occupancy_grid.info.height = 1200
    origin = Pose()
    origin.position.x = -10
    origin.position.y = -10
    origin.orientation.w = 1
    occupancy_grid.info.origin = origin
    occupancy_grid.data = empty_map_flat

    map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=True)
    rate = rospy.Rate(1) # 10hz

    while not rospy.is_shutdown():
        map_pub.publish(occupancy_grid)
        rate.sleep()



