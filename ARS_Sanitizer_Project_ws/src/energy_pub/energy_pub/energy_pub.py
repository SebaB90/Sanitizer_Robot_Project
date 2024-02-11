import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
from rclpy.action import ActionClient
from lifecycle_msgs.srv import GetState
import time
from tf2_ros import Duration

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

        self.energy_map_publisher = self.create_publisher(OccupancyGrid, '/energy_map', qos_profile)

        self.power_map_subscriber = self.create_subscription(OccupancyGrid, '/power_map', self.power_map_callback, qos_profile)
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

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

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.orientation.w = 1.0




    def odom_callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y

    def power_map_callback(self, msg):
        self.power_matrix = msg.data
        self.map_frame_id = msg.header.frame_id
        self.pub_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9       #[s]
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_height = msg.info.height
        self.map_width = msg.info.width

        self.energy_evaluation()
  

    def timer_callback(self):
        if self.energy_matrix  is not None:
            energy_map_msg = OccupancyGrid()
            energy_map_msg.header.frame_id = "/map"
            energy_map_msg.info.width = self.map_width
            energy_map_msg.info.height = self.map_height
            energy_map_msg.info.resolution = self.map_resolution
            energy_map_msg.info.origin.position.x = self.map_origin_x 
            energy_map_msg.info.origin.position.y = self.map_origin_y
            energy_map_msg.data = self.energy_matrix
            energy_map_msg.header.stamp = self.get_clock().now().to_msg()
            self.energy_map_publisher.publish(energy_map_msg)

 
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






    def energy_evaluation(self):

        if self.pub_time  is not None:
            ########## ENERGY DEFINITION ##########
            dt = self.pub_time - self.old_pub_time

            if dt>0:
                if self.first_time_flag is True :
                    self.first_time_flag = False
                    print("sto rifacendo l'azzeramento")
                    self.energy_matrix = [0] * len(self.power_matrix)

                for i in range(len(self.energy_matrix)):
                    self.energy_matrix[i] += int(self.power_matrix[i])
                    self.energy_matrix[i] = min (self.energy_matrix[i], 127)

                self.old_pub_time = self.pub_time


                         
def main(args=None):
    rclpy.init(args=args)
    energy_pub_node = EnergyNavigation()

    energy_pub_node.old_pub_time = time.time()

    rclpy.spin(energy_pub_node)

    energy_pub_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
