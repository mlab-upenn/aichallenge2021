import rclpy
from rclpy.node import Node
import copy
import time

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

from autoware_auto_msgs.msg import Trajectory, TrajectoryPoint, VehicleKinematicState, VehicleStateCommand, VehicleStateReport, VehicleOdometry
from aichallenge_msgs.msg import TimeData
from std_msgs.msg import String
import numpy as np
import os
import pkg_resources

PACKAGE_NAME='upenn_submit'


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.trajectory_pub = self.create_publisher(Trajectory, '/planning/trajectory2', 2)
        self.vehicle_state_pub = self.create_publisher(VehicleStateCommand, '/vehicle/state_command', 1)
        self.current_lane_pub = self.create_publisher(String, '/planning/current_lane', 1)
        
        self.time_delta = time.time_ns()
        self.time_from_start = 0
        self.current_gear = 2
        self.current_speed = 0.0
        self.race_finish = 0
        self.slow = 0

        self.raceline0 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_3.2_dense.npy'))
        self.raceline1 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_-1.2_dense.npy'))
        self.raceline2 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_-5.15_dense.npy'))
        self.raceline3 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/traj_race_cl_4_8_mincur_dense.npy'))
        self.raceline_ind = 0
        self.raceline = self.raceline0

        self.lane_occupied_sub = self.create_subscription(
            String,
            '/planning/lane_occupied',
            self.lane_occupied_callback,
            1)

        self.vehicle_pose_sub = self.create_subscription(
            PoseStamped,
            '/aichallenge/vehicle_pose',
            self.vehicle_pose_callback,
            1)

        self.vehicle_state_sub = self.create_subscription(
            VehicleStateReport,
            '/vehicle/state_report',
            self.vehicle_state_callback,
            1)
        
        self.vehicle_speed_sub = self.create_subscription(
            VehicleOdometry,
            '/vehicle/odometry',
            self.vehicle_speed_callback,
            1
        )

        self.finish_sub = self.create_subscription(
            TimeData,
            '/aichallenge/time',
            self.finish_callback,
            1)

        self.vehicle_pose_sub  # prevent unused variable warning
        self.lane_occupied_sub
        self.vehicle_state_sub
        self.finish_sub
        print("Trajectory Publisher Initialized")
    
    def vehicle_speed_callback(self, msg):
        self.current_speed = msg.velocity_mps

    def vehicle_state_callback(self, msg):
        self.current_gear = msg.gear
        #self.current_rpm = msg.engine_rpm
        #self.current_speed = msg.speed_mps

    def finish_callback(self, msg):
        self.race_finish = msg.hasFinished

    def lane_occupied_callback(self, msg):
        if msg.data == "lane0":
            self.raceline_ind = 0
        if msg.data == "lane1":
            self.raceline_ind = 1
        if msg.data == "lane2":
            self.raceline_ind = 2
        if msg.data == "lane3":
            self.raceline_ind = 3
        if msg.data == "slow":
            self.slow = 2

    def publish_lane_selection(self):
        str_msg = String()
        if self.raceline_ind == 0:
            str_msg.data = "lane0"
        elif self.raceline_ind == 1:
            str_msg.data = "lane1"
        elif self.raceline_ind == 2:
            str_msg.data = "lane2"
        elif self.raceline_ind == 3:
            str_msg.data = "lane3"
        self.current_lane_pub.publish(str_msg)
    
    def vehicle_pose_callback(self, msg):
        def rollover_ind(ind):
            if ind >= self.raceline.shape[0]:
                return ind - self.raceline.shape[0]
            else:
                return ind

        if self.raceline_ind == 0:
            self.raceline = self.raceline0
        elif self.raceline_ind == 1:
            self.raceline = self.raceline1
        elif self.raceline_ind == 2:
            self.raceline = self.raceline2
        elif self.raceline_ind == 3:
            self.raceline = self.raceline3

        if (self.current_gear != 1):
            vehicle_state_msg = VehicleStateCommand()
            vehicle_state_msg.gear = 1
            vehicle_state_msg.mode = 1
            # vehicle_state_msg.blinker = 1
            vehicle_state_msg.stamp = msg.header.stamp
            self.vehicle_state_pub.publish(vehicle_state_msg)
        else:
            if self.current_gear == 1 and self.current_speed>35:
                vehicle_state_msg = VehicleStateCommand()
                vehicle_state_msg.gear = 2
                vehicle_state_msg.mode = 1
                # vehicle_state_msg.blinker = 1
                vehicle_state_msg.stamp = msg.header.stamp
                self.vehicle_state_pub.publish(vehicle_state_msg)
            elif self.current_gear == 2 and self.current_speed<30:
                vehicle_state_msg = VehicleStateCommand()
                vehicle_state_msg.gear = 1
                vehicle_state_msg.mode = 1
                # vehicle_state_msg.blinker = 1
                vehicle_state_msg.stamp = msg.header.stamp
                self.vehicle_state_pub.publish(vehicle_state_msg)
            new_msg = Trajectory()
            new_msg.header.stamp.sec = msg.header.stamp.sec
            new_msg.header.stamp.nanosec = msg.header.stamp.nanosec
            # print('header', msg.header.stamp.sec, new_msg.header.stamp.nanosec)
            new_msg.header.frame_id = 'map'
            current_pose = np.array([msg.pose.position.x, msg.pose.position.y])
            pose_diff = self.raceline[:, 0:2] - current_pose
            distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
            min_distance_ind = np.argmin(distance)
            # for ind in range(len(msg.points)):
            for ind in range(100):
                new_point = TrajectoryPoint()
                new_point.time_from_start.sec = 0
                new_point.time_from_start.nanosec = ind * 10000000
                # print(ind, new_point.time_from_start.sec, new_point.time_from_start.nanosec)
                # new_point.x = msg.points[ind].x
                # new_point.y = msg.points[ind].y
                
                
                # new_point.heading.real = self.traj_arr[np.argmin(distance) + ind, 2]
                # new_point.heading.imag = self.traj_arr[np.argmin(distance) + ind, 3]
                new_point.x = self.raceline[rollover_ind(min_distance_ind + ind), 0]
                new_point.y = self.raceline[rollover_ind(min_distance_ind + ind), 1]

                # calculating heading
                # min_distance_ind_next = min_distance_ind + 1
                # if (min_distance_ind == self.raceline.shape[0]):
                #     min_distance_ind_next = 0
                # yaw = np.arctan2(self.raceline[rollover_ind(min_distance_ind_next + ind), 1] - self.raceline[rollover_ind(min_distance_ind + ind), 1], \
                #     self.raceline[rollover_ind(min_distance_ind_next + ind), 0] - self.raceline[rollover_ind(min_distance_ind + ind), 0])
                # print(yaw)
                # new_point.heading.real = np.cos(yaw / 2)
                # new_point.heading.imag = np.sin(yaw / 2)
                desired_speed = 55.0
                
                    # desired_speed = 0.0
                new_point.longitudinal_velocity_mps = desired_speed
                # new_point.front_wheel_angle_rad = self.traj_arr[np.argmin(distance) + ind, 5]
                # new_point.heading_rate_rps = self.traj_arr[np.argmin(distance) + ind, 6]
                new_msg.points.append(new_point)
            if self.race_finish != 1 and self.slow == 0:
                self.trajectory_pub.publish(new_msg)
                self.publish_lane_selection()
            elif self.slow > 0:
                self.slow -= 1

def main(args=None):
    rclpy.init(args=args)
    print("Trajectory Node Initialized")
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()