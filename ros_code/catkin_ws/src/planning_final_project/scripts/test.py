import rospy

from forest import Forest
from seed import SeedManager
from turtlebot import TurtleBot



if __name__ == "__main__": 
    rospy.init_node("main_node")
    forest = Forest.loadForest("forest100.yaml")
    # SeedManager.spawnForestSeeds(forest)
    # SeedManager.dumpSeedsPosition()
    SeedManager.loadSeeds("seeds100.yaml")

    start_collecting = input("Press enter to start")

    turtlebot = TurtleBot()

    path = [(8.472, 0.55), (9.0, 0.22199999999999998), (9.264, 0.486), (9.528, 0.5000000000000001), (13.0, 9.222000000000005), (13.528, 17.049999999999983), (16.5, 20.0), (22.0, 27.021999999999938), (40.0, 40.0)]

    for waypoint in path: 
        try:
            result = turtlebot.move(waypoint)
            if result: 
                rospy.loginfo("Goal execution done!")
                turtlebot.printCurrentPose()
                # turtlebot.wonderAndCollect()
                # turtlebot.collectNearbySeeds()
                turtlebot.printCurrentPose()
                turtlebot.printSeedsCollection()
                print("==========")

        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    rospy.spin()



