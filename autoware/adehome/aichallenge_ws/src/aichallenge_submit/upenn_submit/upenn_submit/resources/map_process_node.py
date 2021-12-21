# import lanelet2
import os


# dir_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
# map_data_path = os.path.join(dir_path, "data/IndianapolisMotorSpeedway.osm")
# print(map_data_path)

# map = lanelet2.io.load(map_data_path, lanelet2.io.Origin(0, 0))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from autoware_auto_msgs.msg import Trajectory, TrajectoryPoint
from visualization_msgs.msg import MarkerArray
import numpy as np
import ros2_numpy

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/had_maps/viz_had_map',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # self.get_logger().info(msg.markers[0].pose)
        map_arr = []
        left_inds = np.array([0, 3, 6, 9, 12, 15, 18, 21, 24, 27]) + 2
        # left_inds = [2]
        for ind, obj in enumerate(msg.markers):
            print(ind, obj.ns)
            if ind in left_inds:
                # print('')
                # print(ind, obj.header.frame_id)
                for point in msg.markers[ind].points:
                    map_arr.append(ros2_numpy.numpify(point))
        map_arr = np.array(map_arr)
        print(map_arr)
        np.save('center_lane_bound_ori.npy', map_arr)
        exit()
        # print(msg.markers[0].points[0])


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()