import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String

import numpy as np
import ros2_numpy

class MinimalSubscriber(Node):

    def __init__(self):
        self.traj_arr = []
        super().__init__('pointcloud_subscriber')
        self.point_pub = self.create_publisher(PointCloud2, '/perception/points_filered', 1)
        self.lane_pub = self.create_publisher(String, '/planning/lane_occupied', 1)
        self.pause = 0

        self.x_limit = [0, 30]
        self.y_limit = [-2, 2]
        self.z_limit = [-1, 1]

        self.subscription = self.create_subscription(
            PointCloud2,
            '/perception/points_nonground',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning
        print("Pointcloud listener initialized")

    def listener_callback(self, msg):
        
        pc = ros2_numpy.numpify(msg)
        # pc = pc.flatten()
        # pc = pc[np.where( (pc['intensity'] > 100) )]
        pc = pc[np.where( (pc['x'] > self.x_limit[0]) & (pc['x'] < self.x_limit[1]) )]
        pc = pc[np.where( (pc['y'] > self.y_limit[0]) & (pc['y'] < self.y_limit[1]) )]
        pc = pc[np.where( (pc['z'] > self.z_limit[0]) & (pc['z'] < self.z_limit[1]) )]
        pc_arr = ros2_numpy.point_cloud2.get_xyz_points(pc)
        
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

        count = 0
        y_thresh = 1
        x_thresh = 13
        
        if self.pause > 0:
            self.pause -= 1
        else:
            for point in pc_arr:
                if point[0] < x_thresh and np.abs(point[1]) < y_thresh:
                    count += 1
            if count > 10:
                str_msg = String()
                str_msg.data = "occupied"
                self.pause = 100
                self.lane_pub.publish(str_msg)
        
        new_msg = ros2_numpy.msgify(PointCloud2, pc)
        new_msg.header.frame_id = 'base_link'
        self.point_pub.publish(new_msg)
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