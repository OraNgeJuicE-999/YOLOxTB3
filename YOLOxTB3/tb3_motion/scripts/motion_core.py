import roslaunch
import rospy
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

        # Initialize Path
        self.launch_path = "/root/ws/src/tb3_motion/launch/"
        self.detect_lane_path = self.launch_path + 'detect_lane.launch'
        self.control_lane_path = self.launch_path + 'control_lane.launch'
        self.intersection_path = self.launch_path + 'intersection.launch'
        self.construction_path = self.launch_path + 'construction.launch'

        # ROS UUID
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        # Since we only start and stio each node once, I won't use flags to track the running of the launch files

        # Oneshot that start the operation
        rospy.Timer(rospy.Duration(0.1), self.cbStartOperation, oneshot=True)
    
    def cbStartOperation(self, event):
        self.launcher()

    def cbTaskComplete(self, task):
        # Stop Intersection & Start Construction
        if task.data.strip() == "Intersection_Complete" and self.stop_intersection == False:
            self.stop_intersection = True
            self.cbStopIntersection()

        # Stop Construction
        elif task.data.strip() == "Construction_Complete" and self.stop_construction == False:
            self.stop_construction = True
            self.cbStopConstruction()

    def cbStopIntersection(self):
        # Start Construction Launch, After Intersection Launch
        if self.stop_intersection:
            self.construction_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.construction_path])
            self.construction_launch.start()
            self.intersection_launch.shutdown()

    def cbStopConstruction(self):
        # Stop Construction Launch, And stop the operation
        if self.stop_construction:
            self.construction_launch.shutdown()
            rospy.sleep(3)
            self.control_lane_launch.shutdown()
            self.detect_lane_launch.shutdown()

    def launcher(self):
        # Start Launch File
        # Detect Lane & Control Lane
        self.detect_lane_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.detect_lane_path])
        self.control_lane_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.control_lane_path])
        self.detect_lane_launch.start()
        self.control_lane_launch.start()

        # Setup Time Delay
        rospy.sleep(3)

        # Start Intersection Launch
        self.intersection_launch = roslaunch.parent.ROSLaunchParent(self.uuid, [self.intersection_path])
        self.intersection_launch.start()

    def main(self):
        rospy.spin()


        
        

if __name__ == '__main__':
    rospy.init_node('motion_core', anonymous=True)
    node = MotionCore()
    node.main()