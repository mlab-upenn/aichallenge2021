import rclpy
from rclpy.node import Node
import copy
import time

from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

from autoware_auto_msgs.msg import Trajectory, TrajectoryPoint, VehicleKinematicState
import numpy as np
import os
import pkg_resources

PACKAGE_NAME='upenn_submit'


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Trajectory, '/planning/trajectory2', 2)
        timer_period = 0.01  # seconds
        self.i = 0
        self.time_delta = time.time_ns()
        self.time_from_start = 0
        self.raceline0 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_-5_dense.npy'))
        self.raceline1 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_3_dense.npy'))
        self.raceline_ind = 0
        self.raceline = self.raceline0


        self.subscription2 = self.create_subscription(
            String,
            '/planning/lane_occupied',
            self.lane_occupied_callback,
            1)

        self.subscription = self.create_subscription(
            PoseStamped,
            '/aichallenge/vehicle_pose',
            self.listener_callback,
            1)

        # self.subscription = self.create_subscription(
        #     VehicleKinematicState,
        #     '/vehicle/vehicle_kinematic_state',
        #     self.listener_callback,
        #     1)
        self.subscription  # prevent unused variable warning
        self.subscription2
        print("Trajectory Publisher Initialized")

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

    # def timer_callback(self):
    #     msg = Trajectory()
    #     # print(self.i // 100)
    #     msg.header.stamp.sec = (int)(self.i // 100)
    #     msg.header.stamp.nanosec = (int)((self.i % 100) * 0.01 * 1000000000)
    #     msg.header.frame_id = 'map'
    #     for ind in range(40 + msg.header.stamp.sec, 140 + msg.header.stamp.sec):
    #         new_point = TrajectoryPoint()
    #         new_point.time_from_start.sec = (int)(self.i // 100)
    #         new_point.time_from_start.nanosec = (int)((self.i % 100) * 0.01 * 1000000000)
    #         new_point.x = self.traj_arr[ind, 0]
    #         new_point.y = self.traj_arr[ind, 1]
    #         new_point.longitudinal_velocity_mps = 10.0
    #         new_point.acceleration_mps2 = 2.0
    #         msg.points.append(new_point)
    #     self.publisher_.publish(msg)
    #     # self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1

    # def listener_callback(self, msg):
    #     # if self.i == 0 and len(msg.points) > 10:
    #     self.msg = copy.deepcopy(msg)
    #     self.i += 1
    #     # self.msg.points = []
    #     for ind in range(len(msg.points)):
    #         self.msg.points[ind].x = self.traj_arr[(self.i+ind), 0]
    #         self.msg.points[ind].y = self.traj_arr[(self.i+ind), 1]
    #         self.msg.points[ind].longitudinal_velocity_mps = 50.0
    #         #     self.msg.points.append(msg.points[ind])
    #         #     self.first_traj_arr.append([msg.points[ind].x, msg.points[ind].y])
    #         # self.first_traj_arr = np.array(self.first_traj_arr)
    #         # np.save('first_traj_arr.npy', self.first_traj_arr)
    #     print(msg.points[20].time_from_start.nanosec)
    #     self.msg.header.stamp.sec = msg.header.stamp.sec
    #     self.msg.header.stamp.nanosec = msg.header.stamp.nanosec
    #     for ind in range(len(self.msg.points)):
    #         self.msg.points[ind].time_from_start.sec = msg.points[20].time_from_start.sec
    #         self.msg.points[ind].time_from_start.nanosec = msg.points[20].time_from_start.nanosec


    #     self.time_from_start += time.time_ns() - self.time_delta
    #     # print(msg.header.stamp.nanosec)
    #     # if self.time_from_start < 1000000000:
    #     #     self.msg.header.stamp.nanosec = self.time_from_start 
    #     #     for ind in range(len(self.msg.points)):
    #     #         self.msg.points[ind].time_from_start.nanosec = self.time_from_start
    #     # else:
    #     #     self.time_from_start -= 1000000000
    #     #     self.msg.header.stamp.sec += 1
    #     #     self.msg.header.stamp.nanosec = self.time_from_start
    #     #     for ind in range(len(self.msg.points)):
    #     #         self.msg.points[ind].time_from_start.sec += 1
    #     #         self.msg.points[ind].time_from_start.nanosec = self.time_from_start

    #     self.publisher_.publish(self.msg)
    #     self.time_delta = time.time_ns()

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