from aichallenge_ws.src.aichallenge_submit.aichallenge_submit.publish_trajectory import PACKAGE_NAME
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
import copy
import time

from scipy.spatial.transform import Rotation

from geometry_msgs.msg import Pose, PoseStamped, Point32

# from tf2_geometry_msgs import PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud, ChannelFloat32

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from autoware_auto_msgs.msg import (
    Trajectory,
    TrajectoryPoint,
    VehicleKinematicState,
    BoundingBox,
    BoundingBoxArray,
)
import numpy as np
from copy import copy
import pkg_resources

LANE_EPSILON = 40
CENTROID_EPSILON = 3
PACKAGE_NAME = 'upenn_submit'

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher = self.create_publisher(String, "/planning/lane_select", 10)
        self.debug_publisher = self.create_publisher(PointCloud, "/planning/detected_objects", 10)
        
        self.objects = []
        self.lanes = [
            np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_-5_dense.npy')),
            np.load(os.path.join('share', PACKAGE_NAME) + "clean_center_dense.npy"),
            np.load(pkg_resources.resource_filename(PACKAGE_NAME,'resources/clean_lane_3_dense.npy')),
        ]
        self.lane_names = ["left", "center", "right"]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription_one = self.create_subscription(
            PoseStamped, "/aichallenge/vehicle_pose", self.listener_callback, 1
        )

        self.subscription_two = self.create_subscription(
            BoundingBoxArray,
            "/perception/lidar_bounding_boxes_filtered",
            self.obstacle_update_callback,
            1,
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription_one  # prevent unused variable warning
        self.subscription_two

    def obstacle_update_callback(self, msg):
        """
        This function updates the obstacle set that we store in this class instance.
        """
        self.objects = None
        self.objects = msg.boxes  # Should be a list of BoundingBoxes
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("map", "base_link", now)
            #print("Updated Transformation")
        except TransformException as ex:
            self.get_logger().info(f"Could not transform base_link to map: {ex}")
            return

        viz_pc = PointCloud()
        viz_pc.header.stamp = self.get_clock().now().to_msg()
        viz_pc.header.frame_id = 'map'
        viz_pc.channels=[ChannelFloat32()]
        viz_pc.channels[0].name = 'intensity'
        viz_points = list()

        #print("# of Boxes: {}".format(len(self.objects)))

        for i, object in enumerate(self.objects):
            newpt = np.array([object.centroid.x, object.centroid.y, object.centroid.z])
            # Rotate
            r = Rotation.from_quat(
                [
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                ]
            )
            newpt = r.apply(newpt)
            # Translate
            newpt += np.array(
                [
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                ]
            )
            self.objects[i] = newpt
            viz_pt = Point32()
            viz_pt.x, viz_pt.y, viz_pt.z = newpt[0], newpt[1], newpt[2]
            viz_points.append(copy(viz_pt))
            #print("Rotation: {} | Point: {}".format(r.as_dcm(), newpt))
        viz_pc.points = viz_points
        viz_pc.channels[0].values = [255.0 for i in viz_points]
        self.debug_publisher.publish(viz_pc)

    def listener_callback(self, msg):
        #print("Pose updated")
        self.current_pose = np.array([msg.pose.position.x, msg.pose.position.y])

    def timer_callback(self):
        try:
            self.current_pose
            self.objects
        except:
            return  # Prereq items don't exist
        lanes_occupied = [False for lane in self.lanes]
        selected_lane = "center"

        for lane_idx, lane in enumerate(self.lanes):
            # Filter the positions on the lane to only include those within LANE_EPSILON of our position
            pose_diff = lane[:, 0:2] - self.current_pose
            distance = np.sqrt(pose_diff[:, 0] ** 2 + pose_diff[:, 1] ** 2)
            filter = distance < LANE_EPSILON
            masked_lane = lane[filter]
            # Check the distance from the centroid to the min distance point on the lane.
            for centroid in self.objects:
                # box Has type BoundingBox
                object_pose = centroid[:2]
                object_diff = masked_lane[:, 0:2] - object_pose
                object_distance = np.sqrt(
                    object_diff[:, 0] ** 2 + object_diff[:, 1] ** 2
                )
                min_dist = np.min(object_distance)
                if min_dist < CENTROID_EPSILON:
                    lanes_occupied[lane_idx] = True

        # We have a occupancy vector of the lanes. We choose a lane based upon this.
        # Basic strategy is to choose the leftmost lane that is available.
        if all([not i for i in selected_lane]):
            selected_lane = 'center'  # this will be the raceline
        elif not lanes_occupied[0]:
            selected_lane = "left"
        elif not lanes_occupied[1]:
            selected_lane = "center"
        elif not lanes_occupied[2]:
            selected_lane = "right"

        output = String()
        output.data = selected_lane

        print(lanes_occupied, selected_lane)

        self.publisher.publish(output)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    executor = MultiThreadedExecutor(num_threads=3)
    rclpy.spin(minimal_publisher, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
