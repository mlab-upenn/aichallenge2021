import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='upenn_submit',
            node_executable='publish_trajectory',
            node_name='upenn_trajectory_publisher',
            node_namespace='aichallenge'),
        launch_ros.actions.Node(
            package='upenn_submit',
            node_executable='subscribe_pointcloud',
            node_name='upenn_subscribe_pointcloud',
            node_namespace='aichallenge')
    ])