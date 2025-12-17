from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    pkg_description = get_package_share_directory('depthai_examples')
    depthai = os.path.join(pkg_description, 'launch', 'tracker_yolo_spatial_node.launch.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depthai)
        ),
        Node(
            package='robot_perception',
            executable='pose_pub_hw',
            output='screen',
        )
    ]) 

if __name__ == '__main__':
    generate_launch_description() 