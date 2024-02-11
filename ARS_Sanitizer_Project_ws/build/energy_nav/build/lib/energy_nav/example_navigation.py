#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
import math
import numpy as np
#import Duration



from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import OccupancyGrid


import rclpy
from rclpy import duration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
import sys

from tf2_ros import Duration

# Global variables
UV_MIN_LEVEL = 40  #10*[mJ]  (valore riscalato)


class BasicNavigator(Node):
    def __init__(self):
        super().__init__(node_name='basic_navigator')
        self.initial_pose = Pose()
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        # Initialize the robot's pose variables
        self.amcl_pose = PoseWithCovarianceStamped()
        # Variable for the energy map
        self.energy_matrix = OccupancyGrid()
        self.pub_time = None
        self.energy_matrix_received = False


        qos_profile = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        self.initial_pose_received = False
        self.nav_through_poses_client = ActionClient(self,
                                                    NavigateThroughPoses,
                                                    'navigate_through_poses')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                        'amcl_pose',
                                                        self._amclPoseCallback,
                                                        qos_profile)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                    'initialpose',
                                                    10)
        
        # Subscriber to the energy map
        self.energy_map_subscriber = self.create_subscription(OccupancyGrid, '/energy_map', self.energy_map_callback, qos_profile)


    # OUR CALLBACK AND FUNCTIONS ############################################################################
    def energy_map_callback(self, msg):
        if self.initial_pose_received :
            self.energy_matrix.data = msg.data
            self.energy_matrix.header.frame_id = msg.header.frame_id
            self.pub_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9       #[s]
            self.energy_matrix.info.resolution = msg.info.resolution
            self.energy_matrix.info.origin.position.x = msg.info.origin.position.x
            self.energy_matrix.info.origin.position.y  = msg.info.origin.position.y
            self.energy_matrix.info.height = msg.info.height
            self.energy_matrix.info.width  = msg.info.width
            self.energy_matrix_received = True

            
    
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
        return math.sqrt((x - self.amcl_pose.pose.pose.position.x) ** 2 + (y - self.amcl_pose.pose.pose.position.y) ** 2)
    


    def goal_generator (self, room_ids):
        ########## GOAL DEFINITION ##########

        # Acquisizione della mappa 
        if self.energy_matrix_received is True:

            energy_matrix = self.energy_matrix.data
            res = self.energy_matrix.info.resolution
            x_origin = self.energy_matrix.info.origin.position.x
            y_origin = self.energy_matrix.info.origin.position.y
            width = self.energy_matrix.info.width
            height = self.energy_matrix.info.height

            print('generazione inniziata')

            # indici dei punti non disinfettati a sufficienza
            dirty_ids = [i for i, val in enumerate(energy_matrix) if val < UV_MIN_LEVEL and i in room_ids]
            if dirty_ids == []:  # se non ci sono punti sporchi interrompi l'espolarione
                print('esplorazione completata')
                return (0, 0, True)
            # trovo il più vicino
            smallest_dist = 10000
            for id in (dirty_ids):
                mx, my = self.id_to_map_point(id, res, x_origin, y_origin, width)
                dist = self.distance_from_robot(mx, my)

                if  dist < smallest_dist and dist > 1.1:
                    smallest_dist = dist
                    goal_x = mx
                    goal_y = my

            if goal_x == None or goal_y == None:
                print('esplorazione completata')
                return (0, 0, True)
                    
            return (goal_x, goal_y, False)
    
    ####################################################################################################
                         


    def setInitialPose(self, initial_pose):
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self._setInitialPose()

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
            # task was cancelled or completed
            return True
        
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
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

    def _waitForInitialPose(self):
        if self.initial_pose_received:
            self._setInitialPose()
            self.info('amcl_pose received and published')
            print(f"Initial pose received: {self.amcl_pose.pose.pose.position.x} {self.amcl_pose.pose.pose.position.y}")
        
        rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.amcl_pose.pose.pose.position.x = msg.pose.pose.position.x
        self.amcl_pose.pose.pose.position.y = msg.pose.pose.position.y
        self.amcl_pose.header.frame_id = msg.header.frame_id
        # Extracting the angle around z-axis from the quaternion
        self.amcl_pose.pose.pose.orientation = msg.pose.pose.orientation
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def _setInitialPose(self):
        msg = self.amcl_pose
        self.initial_pose_pub.publish(msg)
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


def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    """
    Camabiato da noi il tipo di dato di initial pose (da Pose a PoseStamped) in qiesto modo possiamo
    settare il frame di riferimento --> Da capire se usare odom e map
    """
    # Waiting for amcl_pose to be received
    navigator.info('Waiting for amcl_pose to be received and for the navigation2 stack to become active...')
    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()
    input('Navigation2 ok, enter to continue')


    # Setting the goal pose with our logic
    sanification_complete = False
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    # Definizione coordinate stanze
    room = 0
    room_ids = []

    if room == 0:
        # Kitchen
        x_room_range = list(np.arange(2.6, 7.0, 0.2))
        y_room_range = list(np.arange(0.2, 4.2, 0.2))
    # due for innestati per generare tutte le coordinate della stanza
    for ix in range(len(x_room_range)):
        x = x_room_range[ix]
        for iy in range(len(y_room_range)):
            y = y_room_range[iy]
            id = navigator.map_point_to_id(x, y, navigator.energy_matrix.info.resolution, navigator.energy_matrix.info.origin.position.x, navigator.energy_matrix.info.origin.position.y, navigator.energy_matrix.info.width)
            room_ids.append(id)  

    with open('room_ids.txt', 'w') as file:
    # Iteriamo ogni elemento della lista e lo scriviamo nel file
        for item in room_ids:
            file.write("%s\n" % item)


    while not sanification_complete:
        goal_pose.pose.position.x, goal_pose.pose.position.y, sanification_complete = navigator.goal_generator(room_ids)
        
        if not sanification_complete:
            navigator.goToPose(goal_pose)
            print('Navigator go to pose')
            i = 0
            while not navigator.isNavComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                i = i + 1
                feedback = navigator.getFeedback()
                if feedback and i % 5 == 0:
                    print('Estimated time of arrival: ' + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                        + ' seconds.')            
                    # Some navigation timeout to demo cancellation
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        navigator.cancelNav()
                
    print('goal concluded')
    navigator.destroy_node()
    rclpy.shutdown()    

    exit(0)