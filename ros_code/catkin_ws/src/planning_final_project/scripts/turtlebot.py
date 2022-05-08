import rospy
import actionlib

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry

from seed import Seed, SeedManager


class TurtleBot:
    def __init__(self): 
        self.last_check_time = rospy.Time.now()

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_client.wait_for_server()


        self.current_pose = (0.5, 0.5)
        self.seedsCollection = {}
        self.seedsCollection["red"] = []
        self.seedsCollection["blue"] = []
        self.seedsCollection["black"] = []

        self.mode = "collect"


    def odom_callback(self, msg): 
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pose = (x, y)

        duration = msg.header.stamp - self.last_check_time
        if (duration.to_sec() > 2 and self.mode=="collect"):
            self.last_check_time = rospy.Time.now()
            self.collectSeed(0.2)

    def move(self, goalXY): 
        x, y = goalXY

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1

        self.move_base_client.send_goal(goal)
        wait = self.move_base_client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.move_base_client.get_result()

    def printCurrentPose(self): 
        rospy.loginfo("Turtlebot is at {}".format(self.current_pose))

    def collectSeed(self, dist=0.05): 
        seed = SeedManager.collectSeed(self.current_pose, dist)
        if (seed == None): 
            return

        self.seedsCollection[seed.color].append(seed)

    def collectNearbySeeds(self): 
        rospy.loginfo("Start collecting nearby seeds")
        nearby_distance = 0.2
        while True: 
            seed = SeedManager.getClosestSeed(self.current_pose, dist=nearby_distance)
            if (seed == None): 
                break

            self.move(seed.getXYPosition())
            self.collectSeed()

    def wonderAndCollect(self): 
        TIME_INTERVAL = 0.5
        for i in range(3): 
            cmd_vel_msg = Twist() 
            cmd_vel_msg.linear.x = 0.1
            cmd_vel_msg.angular.z = 0.1

            self.cmd_vel_pub.publish(cmd_vel_msg)
            rospy.sleep(TIME_INTERVAL)
            self.collectSeed()


    def printSeedsCollection(self): 
        red_count = len(self.seedsCollection["red"])
        blue_count = len(self.seedsCollection["blue"])
        black_count = len(self.seedsCollection["black"])

        rospy.loginfo("Turtlebot has seeds [red: {}, blue: {}, black: {}]" \
                       .format(red_count, blue_count, black_count))

    def startPlanting(self): 
        self.mode = "plant"
    

if __name__ == "__main__":
    rospy.init_node("turtlebot_node", anonymous=True)

    seed1 = Seed("red", (3, 3))
    SeedManager.spawn(seed1)
    rospy.sleep(1)

    try:
        turtlebot = TurtleBot()
        goal = (3, 3)
        result = turtlebot.move(goal)
        if result: 
            rospy.loginfo("Goal execution done!")
            turtlebot.printCurrentPose()
            turtlebot.wonderAndCollect()
            # turtlebot.collectSeed()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

    rospy.spin()







