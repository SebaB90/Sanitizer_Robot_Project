import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
import time

# Power in [µW*m2]
PI = 100
# Max range for UV irradiation [m]
MAX_INFLUENCE_RADIUS = 5

class PowerMatrix (Node):
    def __init__(self):
        super().__init__('PowerMatrix')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_subscriber = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.laser_subscriber = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.power_map_publisher = self.create_publisher(OccupancyGrid, '/power_map', qos_profile)


        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.poly = []
        self.x_robot = None
        self.y_robot = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.map_frame_id = None
        self.map_orientation = None
        self.map_data = None  # Initialize map_data to None
        self.power_resolution = 0.15
        self.power_width = 0
        self.power_height = 0

        self.power_matrix = []

    def scan_callback(self, msg):
        if self.x_robot is not None and self.y_robot is not None and self.map_data is not None:
            self.poly = []
            angle = (msg.angle_min) % (2 * math.pi)  # Normalize the angle
            for distance in msg.ranges:
                distance = min(distance, MAX_INFLUENCE_RADIUS)
                # Trasformazione delle coordinate locali in globali
                x_rob_frame = distance * math.cos(angle) 
                y_rob_frame = distance * math.sin(angle) 
                x_global_frame = math.cos(-self.yaw_robot)*x_rob_frame + math.sin(-self.yaw_robot)*y_rob_frame + self.x_robot
                y_global_frame = -math.sin(-self.yaw_robot)*x_rob_frame + math.cos(-self.yaw_robot)*y_rob_frame + self.y_robot
                self.poly.append((x_global_frame, y_global_frame))
                
                angle += msg.angle_increment
                # Mantenere l'angolo all'interno dell'intervallo [-2pi, 2pi]
                angle = (angle + 2 * math.pi) % (2 * math.pi)
            self.create_power_matrix()

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def odom_callback(self, msg):
        self.x_robot = msg.pose.pose.position.x
        self.y_robot = msg.pose.pose.position.y
        # Extracting the angle around z-axis from the quaternion
        self.robot_orientation = msg.pose.pose.orientation
        self.roll_robot, self.pich_robot, self.yaw_robot = self.euler_from_quaternion(self.robot_orientation)
        

    def map_callback(self, msg):
        self.map_data = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_orientation = msg.info.origin.orientation
        self.map_frame_id = msg.header.frame_id
    

    def timer_callback(self):
        if self.map_frame_id is not None and len(self.power_matrix) > 0:
            power_map_msg = OccupancyGrid()
            power_map_msg.header.frame_id = "/map"
            power_map_msg.info.width = int(self.map_width * self.map_resolution / self.power_resolution)
            power_map_msg.info.height = int(self.map_height * self.map_resolution / self.power_resolution)
            power_map_msg.info.resolution = self.power_resolution
            power_map_msg.info.origin.position.x = self.map_origin_x 
            power_map_msg.info.origin.position.y = self.map_origin_y
            #power_map_msg.info.origin.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            power_map_msg.data = self.power_matrix
            power_map_msg.header.stamp = self.get_clock().now().to_msg()
            self.power_map_publisher.publish(power_map_msg)

    def is_inside_polygon(self, x, y):
        n = len(self.poly)
        inside = False
        p1x, p1y = self.poly[0]
        for i in range(n + 1):
            p2x, p2y = self.poly[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def create_power_matrix(self):
        self.power_height = int(self.map_height*self.map_resolution/self.power_resolution)
        self.power_width = int(self.map_width*self.map_resolution/self.power_resolution)

        self.power_matrix = [0] * (self.power_height * self.power_width)

        for id in range(self.power_height * self.power_width ):
            mx = (id % self.power_width)*self.power_resolution  + self.map_origin_x
            my = (id // self.power_width)* self.power_resolution  + self.map_origin_y
            dist = self.distance_from_robot(mx, my)

            if self.is_inside_polygon(mx, my) and dist > 0.1:
                # Power in [µW] - 126
                 self.power_matrix[id] = min(max(int(2*PI / (dist ** 2)-50), -126), 126)


    def distance_from_robot(self, x, y):
        return math.sqrt((x - self.x_robot) ** 2 + (y - self.y_robot) ** 2)
    




    


def main(args=None):
    rclpy.init(args=args)
    power_matrix_node = PowerMatrix()
    print("RUNNING:")
    rclpy.spin(power_matrix_node)
    power_matrix_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
