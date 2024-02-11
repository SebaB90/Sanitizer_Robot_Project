import math
import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
import time
from tf2_ros import Duration

from action_msgs.msg import GoalStatus

UV_MIN_LEVEL = 100  #10*[mJ]

class EnergyNavigation (Node):
    def __init__(self):
        super().__init__('EnergyNavigation')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.first_time_flag = True

        
        self.energy_map_subscriber = self.create_subscription(OccupancyGrid, '/energy_map', self.energy_map_callback, qos_profile)
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')


        self.power_matrix = None
        self.map_frame_id = None
        self.pub_time = None
        self.old_pub_time = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_height = None
        self.map_width = None
        self.energy_matrix = None


        self.x_robot = None
        self.y_robot = None

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.orientation.w = 1.0




    def odom_callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y

    def energy_map_callback(self, msg):
        if self.x_robot is not None:
            self.energy_matrix = msg.data
            self.map_frame_id = msg.header.frame_id
            self.pub_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9       #[s]
            self.map_resolution = msg.info.resolution
            self.map_origin_x = msg.info.origin.position.x
            self.map_origin_y = msg.info.origin.position.y
            self.map_height = msg.info.height
            self.map_width = msg.info.width

        self.goal_generator()


    def goToPose(self, pose):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                    self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                        str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True


    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return
    
    def isNavComplete(self):
        if not self.result_future:
            return True
        
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            print("Attendo")
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True
    
    def getFeedback(self):
        return self.feedback


    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return
    
    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


    def goal_generator (self):
        ########## GOAL DEFINITION ##########

        if self.energy_matrix is not None:

            print('generazione inniziata')

            # indici dei punti non disinfettati a sufficienza
            dirty_ids = [i for i, val in enumerate(self.energy_matrix) if val < UV_MIN_LEVEL]
            # trovo il più vicino
            smallest_dist = 10000
            for id in range(len(dirty_ids)):
                mx, my = self.id_to_map_point(id, self.map_resolution, self.map_origin_x, self.map_origin_y, self.map_width)
                dist = self.distance_from_robot(mx, my)

                if  dist < smallest_dist and dist > 1.5:

                    smallest_dist = dist
                    goal_x = mx
                    goal_y = my
                    

            self.goal_pose.pose.position.x = goal_x
            self.goal_pose.pose.position.y = goal_y
            
            self.goToPose(self.goal_pose)

            i = 0
            while not self.isNavComplete():
                print("aspetto....")
                i = i + 1
                feedback = self.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')            
                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                        self.cancelNav()
                        
            print('Waypoint Reached')
            
            return True

        
        


    
    def id_to_map_point(self, id, res, x_origin, y_origin, width):
        mx = (id % width)*res  + x_origin
        my = (id // width)* res  + y_origin

        return(mx, my)
    
    def map_point_to_id (self, mx, my, res, x_origin, y_origin, width):
        col = int((mx - x_origin) / res)
        row = int((my - y_origin) / res)
        id = row * width + col
        return id

    def distance_from_robot(self, x, y):
        return math.sqrt((x - self.x_robot) ** 2 + (y - self.y_robot) ** 2)
                         
                         
def main(argv=sys.argv[1:]):
    rclpy.init()
    energy_nav_node = EnergyNavigation()
    print("NAVIGATING:")

    input('Navigation2 ok, enter to continue')


    rclpy.spin(energy_nav_node)
    

    energy_nav_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
