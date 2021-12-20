import rclpy
from rclpy.node import Node
import copy
import time

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

from autoware_auto_msgs.msg import Trajectory, TrajectoryPoint, VehicleKinematicState, VehicleStateCommand, VehicleStateReport
import numpy as np
import os
import pkg_resources

PACKAGE_NAME='upenn_submit'


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Trajectory, '/planning/trajectory2', 2)
        self.vehicle_state_pub = self.create_publisher(VehicleStateCommand, '/vehicle/state_command', 1)

        timer_period = 0.01  # seconds
        self.i = 0
        self.time_delta = time.time_ns()
        self.time_from_start = 0
        self.raceline0 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_-5_dense.npy'))
        self.raceline1 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_3_dense.npy'))
        self.raceline_ind = 0
        self.raceline = self.raceline0
        self.current_gear = 2


        self.lane_occupied_sub = self.create_subscription(
            String,
            '/planning/lane_occupied',
            self.lane_occupied_callback,
            1)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/aichallenge/vehicle_pose',
            self.listener_callback,
            1)

        self.vehicle_state_sub = self.create_subscription(
            VehicleStateReport,
            '/vehicle/state_report',
            self.vehicle_state_callback,
            1)

        # self.subscription = self.create_subscription(
        #     VehicleKinematicState,
        #     '/vehicle/vehicle_kinematic_state',
        #     self.listener_callback,
        #     1)
        self.subscription  # prevent unused variable warning
        self.lane_occupied_sub
        self.vehicle_state_sub
        print("Trajectory Publisher Initialized")

    def vehicle_state_callback(self, msg):
        self.current_gear = msg.gear

    def lane_occupied_callback(self, msg):
        if msg.data == "occupied":
            if self.raceline_ind == 0:
                self.raceline_ind = 1
            elif self.raceline_ind == 1:
                self.raceline_ind = 0
    
    def listener_callback(self, msg):
        def rollover_ind(ind):
            if ind >= self.raceline.shape[0]:
                return ind - self.raceline.shape[0]
            else:
                return ind

        if self.raceline_ind == 0:
            self.raceline = self.raceline0
        elif self.raceline_ind == 1:
            self.raceline = self.raceline1

        if (self.current_gear != 1):
            vehicle_state_msg = VehicleStateCommand()
            vehicle_state_msg.gear = 1
            vehicle_state_msg.mode = 0
            # vehicle_state_msg.blinker = 1
            vehicle_state_msg.stamp = msg.header.stamp
            self.vehicle_state_pub.publish(vehicle_state_msg)
        else:

            new_msg = Trajectory()
            new_msg.header.stamp.sec = msg.header.stamp.sec
            new_msg.header.stamp.nanosec = msg.header.stamp.nanosec
            # print('header', msg.header.stamp.sec, new_msg.header.stamp.nanosec)
            new_msg.header.frame_id = 'map'
            current_pose = np.array([msg.pose.position.x, msg.pose.position.y])
            pose_diff = self.raceline[:, 0:2] - current_pose
            distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
            # for ind in range(len(msg.points)):
            for ind in range(100):
                new_point = TrajectoryPoint()
                new_point.time_from_start.sec = 0
                new_point.time_from_start.nanosec = ind * 10000000
                # print(ind, new_point.time_from_start.sec, new_point.time_from_start.nanosec)
                # new_point.x = msg.points[ind].x
                # new_point.y = msg.points[ind].y
                min_distance_ind = np.argmin(distance)
                
                # new_point.heading.real = self.traj_arr[np.argmin(distance) + ind, 2]
                # new_point.heading.imag = self.traj_arr[np.argmin(distance) + ind, 3]
                new_point.x = self.raceline[rollover_ind(min_distance_ind + ind), 0]
                new_point.y = self.raceline[rollover_ind(min_distance_ind + ind), 1]

                # calculating heading
                min_distance_ind_next = min_distance_ind + 1
                if (min_distance_ind == self.raceline.shape[0]):
                    min_distance_ind_next = 0
                yaw = np.arctan2(self.raceline[rollover_ind(min_distance_ind_next + ind), 1] - self.raceline[rollover_ind(min_distance_ind + ind), 1], \
                    self.raceline[rollover_ind(min_distance_ind_next + ind), 0] - self.raceline[rollover_ind(min_distance_ind + ind), 0])
                
                # new_point.heading.real = np.cos(yaw / 2)
                # new_point.heading.imag = np.sin(yaw / 2)
                new_point.longitudinal_velocity_mps = 35.0
                # new_point.front_wheel_angle_rad = self.traj_arr[np.argmin(distance) + ind, 5]
                # new_point.heading_rate_rps = self.traj_arr[np.argmin(distance) + ind, 6]
                new_msg.points.append(new_point)
            self.publisher_.publish(new_msg)

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