import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

from tf2_ros import TransformException, Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation

import numpy as np
import ros2_numpy
from time import sleep
import pkg_resources

PACKAGE_NAME='upenn_submit'

class MinimalSubscriber(Node):

    def __init__(self):
        self.traj_arr = []
        super().__init__('pointcloud_subscriber')
        self.point_pub = self.create_publisher(PointCloud2, '/perception/points_filered', 1)
        self.lane_pub = self.create_publisher(String, '/planning/lane_occupied', 1)
        self.raceline0 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_3.3_dense.npy'))
        self.raceline1 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_0_dense.npy'))
        self.raceline2 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_-5_dense.npy'))
        self.raceline3 = np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/traj_race_cl_4_8_mincur_dense.npy'))
        self.lane_names = ['right', 'center', 'left', 'race']
        self.pause = 0
        self.raceline_ind = 0
        self.raceline = self.raceline0
        self.current_pose = np.zeros((2))
        self.timer = 0
        self.timer2 = 0
        self.pre_lane_occupancy = np.array([100, 100, 100, 100])
        self.need_transform = True

        self.r = Rotation.from_quat(
                [
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ]
            )
        self.t = np.array(
                [
                    0.0,
                    0.0,
                    0.0,
                ]
            )
        

        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x_limit = [-10, 100] 
        self.y_limit = [[-2, 14], [-12, 12], [-14, 2], [-12, 12]]
        self.z_limit = [-0.5, 0.9]

        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/perception/points_nonground',
            self.pc_callback,
            1)

        self.current_lane_sub = self.create_subscription(
            String,
            '/planning/current_lane',
            self.current_lane_callback,
            1)

        self.vehicle_pose_sub = self.create_subscription(
            PoseStamped,
            '/aichallenge/vehicle_pose',
            self.vehicle_pose_callback,
            1)

        self.pc_sub  # prevent unused variable warning
        self.current_lane_callback
        print("Pointcloud listener initialized")
    
    def current_lane_callback(self, msg):
        if msg.data == "lane0":
            self.raceline_ind = 0
        elif msg.data == "lane1":
            self.raceline_ind = 1
        elif msg.data == "lane2":
            self.raceline_ind = 2
        elif msg.data == "lane3":
            self.raceline_ind = 3
        # print(self.raceline_ind)

    def vehicle_pose_callback(self, msg):
        self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y])
        if self.need_transform:
            self.r = Rotation.from_quat(
                    [
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w,
                    ]
                ).inv()
            self.t = np.array(
                    [
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z,
                    ]
                )
            self.need_transform = False

    def tranform_map_to_base(self, arr_in):
        # try:
        #     now = rclpy.time.Time()
        #     trans = self.tf_buffer.lookup_transform('base_link', 'map', now, timeout=Duration(seconds=1.0))
        # except TransformException as ex:
        #     self.get_logger().info(f"Could not transform base_link to map: {ex}")
        # ret_arr = []
        # for ind in range(arr_in.shape[0]):
        #     newpt = np.array([arr_in[ind, 0], arr_in[ind, 1], 1])
        #     # Rotate
        #     r = Rotation.from_quat(
        #         [
        #             trans.transform.rotation.x,
        #             trans.transform.rotation.y,
        #             trans.transform.rotation.z,
        #             trans.transform.rotation.w,
        #         ]
        #     )
        #     newpt = r.apply(newpt)
        #     # Translate
        #     newpt += np.array(
        #         [
        #             trans.transform.translation.x,
        #             trans.transform.translation.y,
        #             trans.transform.translation.z
        #         ]
        #     )
        #     ret_arr.append(newpt[0:2])
        # ret_arr = np.array(ret_arr)
        # return ret_arr
        ret_arr = []
        for ind in range(arr_in.shape[0]):
            newpt = np.array([arr_in[ind, 0], arr_in[ind, 1], 1])
            newpt -= self.t
            newpt = self.r.apply(newpt)
            ret_arr.append(newpt[0:2])
        ret_arr = np.array(ret_arr)
        return ret_arr

    def pc_callback(self, msg):
        self.timer += 1
        def rollover_ind(ind_, raceline):
            if ind_ >= raceline.shape[0]:
                ret = ind_ - raceline.shape[0]
            elif ind_ < 0:
                ret = ind_ + raceline.shape[0]
            else:
                ret = ind_
            # print(ind_, ret, raceline.shape[0])
            return ret

        def append_list(min_distance_ind, source_list, look_back_num, look_ahead_num):
            ret_arr = []
            for ind in range(look_back_num, look_ahead_num):
                ret_arr.append(source_list[rollover_ind(min_distance_ind + ind, source_list), :])
            return np.array(ret_arr)

        lane_min_distances = []
        pose_diff = self.raceline0[:, 0:2] - self.current_pose
        distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
        min_distance_ind = np.argmin(distance)
        lane_min_distances.append(distance[min_distance_ind])
        # print(min_distance_ind)
        look_back_num = -10
        look_ahead_num = 70
        raceline0_section = append_list(min_distance_ind, self.raceline0, look_back_num, look_ahead_num)
        raceline0_section = self.tranform_map_to_base(raceline0_section)
        
        pose_diff = self.raceline1[:, 0:2] - self.current_pose
        distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
        min_distance_ind = np.argmin(distance)
        lane_min_distances.append(distance[min_distance_ind])
        raceline1_section = append_list(min_distance_ind, self.raceline1, look_back_num, look_ahead_num)
        raceline1_section = self.tranform_map_to_base(raceline1_section)
        
        pose_diff = self.raceline2[:, 0:2] - self.current_pose
        distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
        min_distance_ind = np.argmin(distance)
        lane_min_distances.append(distance[min_distance_ind])
        raceline2_section = append_list(min_distance_ind, self.raceline2, look_back_num, look_ahead_num)
        raceline2_section = self.tranform_map_to_base(raceline2_section)

        pose_diff = self.raceline3[:, 0:2] - self.current_pose
        distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
        min_distance_ind = np.argmin(distance)
        raceline3_section = append_list(min_distance_ind, self.raceline3, look_back_num, look_ahead_num)
        raceline3_section = self.tranform_map_to_base(raceline3_section)
        
        # if self.raceline_ind == 3:
        lane_min_distances = np.array(lane_min_distances)
        effective_lane = np.argmin(lane_min_distances)

        pc = ros2_numpy.numpify(msg)
        # pc = pc.flatten()
        # pc = pc[np.where( (pc['intensity'] > 100) )]
        pc['intensity'] = np.zeros(pc['intensity'].shape)
        pc = pc[np.where( (pc['x'] > self.x_limit[0]) & (pc['x'] < self.x_limit[1]) )]
        y_limit = self.y_limit[self.raceline_ind]
        pc = pc[np.where( (pc['y'] > y_limit[0]) & (pc['y'] < y_limit[1]) )]
        pc = pc[np.where( (pc['z'] > self.z_limit[0]) & (pc['z'] < self.z_limit[1]) )]
        pc_arr = ros2_numpy.point_cloud2.get_xyz_points(pc)
        
        circle_distance = 1.5
        lane_occupancy = np.array([100, 100, 100, 100])
        if raceline0_section is not None:
            lane_occupancy[0] = 0
            point_inds = np.array([])
            for raceline_ind in range(raceline0_section.shape[0]):
                # print(raceline_section[raceline_ind, :].shape)
                point_inds = np.append(point_inds, np.where(np.linalg.norm(pc_arr[:, 0:2] - raceline0_section[raceline_ind, :], axis=1) < circle_distance)[0])
            if point_inds.shape[0] != 0:
                point_inds = np.unique(point_inds).astype(int)
                pc['intensity'][point_inds] = 40
                lane_occupancy[0] = point_inds.shape[0]
        
        if raceline1_section is not None:
            lane_occupancy[1] = 0
            point_inds = np.array([])
            for raceline_ind in range(raceline1_section.shape[0]):
                # print(raceline_section[raceline_ind, :].shape)
                point_inds = np.append(point_inds, np.where(np.linalg.norm(pc_arr[:, 0:2] - raceline1_section[raceline_ind, :], axis=1) < circle_distance)[0])
            if point_inds.shape[0] != 0:
                point_inds = np.unique(point_inds).astype(int)
                pc['intensity'][point_inds] = 60
                lane_occupancy[1] = point_inds.shape[0]

        if raceline2_section is not None:
            lane_occupancy[2] = 0
            point_inds = np.array([])
            for raceline_ind in range(raceline2_section.shape[0]):
                # print(raceline_section[raceline_ind, :].shape)
                point_inds = np.append(point_inds, np.where(np.linalg.norm(pc_arr[:, 0:2] - raceline2_section[raceline_ind, :], axis=1) < circle_distance)[0])
            if point_inds.shape[0] != 0:
                point_inds = np.unique(point_inds).astype(int)
                pc['intensity'][point_inds] = 80
                lane_occupancy[2] = point_inds.shape[0]
        
        if raceline3_section is not None:
            lane_occupancy[3] = 0
            point_inds = np.array([])
            for raceline_ind in range(raceline3_section.shape[0]):
                # print(raceline_section[raceline_ind, :].shape)
                point_inds = np.append(point_inds, np.where(np.linalg.norm(pc_arr[:, 0:2] - raceline3_section[raceline_ind, :], axis=1) < circle_distance)[0])
            if point_inds.shape[0] != 0:
                point_inds = np.unique(point_inds).astype(int)
                pc['intensity'][point_inds] = 100
                lane_occupancy[3] = point_inds.shape[0]

        
        # print(lane_occupancy, effective_lane)
        pause_time = 5
        min_count = 5
        min_switch_count = 3
        if self.pause > 0:
            self.pause -= 1
        else:
            str_msg = String()
            old_raceline_ind = self.raceline_ind
            raceline_occupied = lane_occupancy > min_count
            print(raceline_occupied)
            if not np.any(raceline_occupied):
                # No racelines occupied Choose optimal line
                str_msg.data = 'lane3'
                if old_raceline_ind != 3:
                    self.pause = pause_time
            
            # Is our current raceline occupied?
            else:
                if not raceline_occupied[effective_lane]:
                    str_msg.data = 'lane{}'.format(effective_lane)
                elif not raceline_occupied[3]:
                    str_msg.data = 'lane3'
                    self.pause = pause_time
                # We search for the closest unoccupied raceline
                else:
                    self.pause = pause_time
                    if effective_lane == 0:
                        if not raceline_occupied[1]:
                            str_msg.data = 'lane1'
                        elif not raceline_occupied[2]:
                            str_msg.data = 'lane2'
                    if effective_lane == 1:
                        if not raceline_occupied[2]:
                            str_msg.data = 'lane2'
                        elif not raceline_occupied[0]:
                            str_msg.data = 'lane0'
                    if effective_lane == 2:
                        if not raceline_occupied[1]:
                            str_msg.data = 'lane1'
                        elif not raceline_occupied[0]:
                            str_msg.data = 'lane0'
                    
            self.lane_pub.publish(str_msg)
            print(str_msg.data)

        self.pre_lane_occupancy = lane_occupancy.copy()
        # print(self.timer)
        # if lane_occupancy[2] == 0 and self.timer > 200 and self.timer < 203:
        #     str_msg = String()
        #     str_msg.data = "lane2"
        #     self.lane_pub.publish(str_msg)
        

        new_msg = ros2_numpy.msgify(PointCloud2, pc)
        new_msg.header.frame_id = 'base_link'
        self.point_pub.publish(new_msg)
        self.need_transform = True
        #sleep(self.pause)
        #self.pause = 0
        # self.point_pub(ros2_numpy.point_cloud2.array_to_pointcloud2(pc, msg.header.stamp, msg.header.frame_id))
        

def main(args=None):
    rclpy.init(args=args)
    print("Pointcloud init")
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


## clustering
# centoids = np.ones((10, 3)) * 999
# counts = np.zeros(10)
# min_distance = 2
# for point_ind, point in enumerate(pc_arr):
#     pc['intensity'][point_ind] = 0
#     pose_diff = np.linalg.norm(centoids - point, axis=1)
#     print(pose_diff)
#     if np.min(pose_diff) > min_distance:
#         for count_ind in range(len(counts)):
#             if counts[count_ind] == 0:
#                 centoids[count_ind] = point
#                 counts[count_ind] += 1
#                 pc['intensity'][point_ind] = count_ind * 20 + 200
#                 break
#     if np.min(pose_diff) < min_distance:
#         centoid_ind = np.argmin(pose_diff)
#         centoids[centoid_ind] = (centoids[centoid_ind, :] + point) /2
#         counts[centoid_ind] += 1
#         print(centoid_ind * 20 + 200)
#         pc['intensity'][point_ind] = centoid_ind * 20 + 500            
# print(pc_arr.shape)
# print(centoids)
# print(counts)