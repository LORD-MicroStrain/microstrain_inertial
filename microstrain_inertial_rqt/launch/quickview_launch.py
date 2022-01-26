import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
      return LaunchDescription([
            # Declare arguments with default values
            DeclareLaunchArgument('driver_namespace', default_value='/'),

            # Substitute environment variable with the passed argument
            SetEnvironmentVariable('MICROSTRAIN_INERTIAL_RQT_NODE_NAME', LaunchConfiguration('driver_namespace')),

            # ****************************************************************** 
            # Quickview Node
            # ****************************************************************** 
            Node(
                  name="microstrain_inertial_quickview",
                  package="rqt_gui",
                  executable="rqt_gui",
                  arguments=[
                        "--force-discover",
                        "--perspective-file",
                        os.path.join(get_package_share_directory('microstrain_inertial_rqt'), 'microstrain_inertial_rqt_common/resource/quickview.perspective'),
                  ]
            )
      ])
  

 
 
