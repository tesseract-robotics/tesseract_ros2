import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import launch
import launch_ros.actions

def generate_launch_description():

    if not "tesseract_collision" in os.environ["AMENT_PREFIX_PATH"]:
        head, tail = os.path.split(get_package_prefix('tesseract_monitoring'))
        path = os.path.join(head, 'tesseract_collision')
        os.environ["AMENT_PREFIX_PATH"] += os.pathsep + path
        print(os.environ["AMENT_PREFIX_PATH"])

    urdf = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur10_robot.urdf')
    srdf = os.path.join(get_package_share_directory('ur_description'), 'urdf', 'ur10_robot.srdf')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            node_name='environment_monitor',
            package='tesseract_monitoring',
            node_executable='tesseract_monitoring_environment_node',
            output='screen',
            arguments=[],
            parameters=[{'desc_param': 'robot_description',
            'robot_description': urdf,
            'robot_description_semantic': srdf}]),
        launch_ros.actions.Node(
            node_name='joint_state_pub',
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            output='screen',
            arguments=[urdf],
            parameters=[{'use_gui': 'false'}]),
    ])
