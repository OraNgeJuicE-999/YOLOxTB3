import roslaunch
import rospy, rospkg
from std_msgs.msg import Bool, String

class MotionCore():
    def __init__(self):
        self.task_complete = rospy.Subscriber('/task/complete', String, self.cbTaskComplete, queue_size=1)

        # Stopping flag
        self.stop_intersection = False
        self.stop_construction = False

        # Initialize Launch 
        self.detect_lane_launch = None
        self.control_lane_launch = None
        self.intersection_launch = None
        self.construction_launch = None
        
        self.started = False
        # Find package 
        rp = rospkg.RosPack()
        self.pkg_path = rp.get_path('tb3_motion')

        # Initialize Paths
        self.detect_lane_path = f'{self.pkg_path}/launch/detect_lane.launch'
        self.control_lane_path = f'{self.pkg_path}/launch/control_lane.launch'
        self.intersection_path = f'{self.pkg_path}/launch/intersection.launch'
        self.construction_path = f'{self.pkg_path}/launch/construction.launch'
        # Since we only start and stio each node once, I won't use flags to track the running of the launch files
        # ROS UUID
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        # Oneshot that start the operation
        # rospy.Timer(rospy.Duration(0.1), self.cbStartOperation, oneshot=True)
    
    # def cbStartOperation(self, event):
        # self.launcher()

    def cbTaskComplete(self, task):
        # Stop Intersection & Start Construction
        if task.data.strip() == "Intersection_Complete" and self.stop_intersection == False:
            self.stop_intersection = True
            rospy.loginfo("Detected -Intersection")
            self.cbStopIntersection()

        # Stop Construction
        elif task.data.strip() == "Construction_Complete" and self.stop_construction == False:
            self.stop_construction = True
            rospy.loginfo("Detected -Construction")
            self.cbStopConstruction()

    def cbStopIntersection(self):
        # Start Construction Launch, After Intersection Launch
        if self.stop_intersection:
            self.construction_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.construction_path])
            self.construction_launch.start()
            rospy.loginfo("Construction :)")
            self.intersection_launch.shutdown()
            rospy.loginfo("Intersection :()")

    def cbStopConstruction(self):
        # Stop Construction Launch, And stop the operation
        if self.stop_construction:
            self.construction_launch.shutdown()
            rospy.loginfo("Construction :(")
            rospy.sleep(3)
            self.control_lane_launch.shutdown()
            self.detect_lane_launch.shutdown()
            rospy.loginfo("Detect & Control :(")

    def launcher(self):
        # Start Launch File
        # Detect Lane & Control Lane
        if self.started == True:
            return
        self.started = True

        rospy.loginfo("Start :)")
        self.detect_lane_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.detect_lane_path])
        self.control_lane_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.control_lane_path])
        self.detect_lane_launch.start()
        self.control_lane_launch.start()
        rospy.loginfo("Detect & Control :)")

        # Setup Time Delay
        rospy.sleep(3)

        # Start Intersection Launch
        self.intersection_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.intersection_path])
        self.intersection_launch.start()
        rospy.loginfo("Intersection :)")

    def main(self):
        rospy.spin()


        
        

if __name__ == '__main__':
    rospy.init_node('motion_core', anonymous=True)
    node = MotionCore()

    rospy.sleep(0.1)
    node.launcher()
    node.main()