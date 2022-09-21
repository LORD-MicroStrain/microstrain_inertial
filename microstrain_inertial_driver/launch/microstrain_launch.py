# Standalone example launch file for GX3, GX4, GX/CX5, RQ1 and GQ7 series devices
# Note: Feature support is device-dependent and some of the following settings may have no affect on your device.
# Please consult your device's documentation for supported features

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, EmitEvent
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory

_PACKAGE_NAME = 'microstrain_inertial_driver'
_DEFAULT_PARAMS_FILE = os.path.join(
  get_package_share_directory(_PACKAGE_NAME),
  'microstrain_inertial_driver_common',
  'config',
  'params.yml'
)
_EMPTY_PARAMS_FILE = os.path.join(
  get_package_share_directory(_PACKAGE_NAME),
  'config',
  'empty.yml'
)

def generate_launch_description():

  # Declare arguments with default values
  launch_description = []
  launch_description.append(DeclareLaunchArgument('namespace',   default_value='/',                description='Namespace to use when launching the nodes in this launch file'))
  launch_description.append(DeclareLaunchArgument('node_name',   default_value=_PACKAGE_NAME,      description='Name to give the Microstrain Inertial Driver node'))
  launch_description.append(DeclareLaunchArgument('configure',   default_value='false',            description='Whether or not to configure the node on startup'))
  launch_description.append(DeclareLaunchArgument('activate',    default_value='false',            description='Whether or not to activate the node on startup'))
  launch_description.append(DeclareLaunchArgument('debug',       default_value='false',            description='Whether or not to log debug information.'))
  launch_description.append(DeclareLaunchArgument('params_file', default_value=_EMPTY_PARAMS_FILE, description='Path to file that will load additional parameters'))

  # Add some old launch parameters for backwards compatibility
  # NOTE: These parameters are deprecated and will be removed in a future release. It is strongly recommended to use the new params_file option instead
  launch_description.append(DeclareLaunchArgument('port',                  default_value='/dev/ttyACM0',        description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('aux_port',              default_value='/dev/ttyACM1',        description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('baudrate',              default_value='115200',              description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('imu_frame_id',          default_value='sensor',              description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('imu_data_rate',         default_value='100.0',                 description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('filter_data_rate',      default_value='10.0',                  description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('gnss1_frame_id',        default_value='gnss1_antenna_wgs84', description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('gnss2_frame_id',        default_value='gnss2_antenns_wgs84', description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('filter_frame_id',       default_value='sensor_wgs84',        description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('filter_child_frame_id', default_value='sensor',              description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('nmea_frame_id',         default_value='nmea',                description="DEPRECATED. Use params_file instead"))
  launch_description.append(DeclareLaunchArgument('use_enu_frame',         default_value='False',               description="DEPRECATED. Use params_file instead"))

  # Pass an environment variable to the node to determine if it is in debug or not
  launch_description.append(SetEnvironmentVariable('MICROSTRAIN_INERTIAL_DEBUG', value=LaunchConfiguration('debug')))

  # ****************************************************************** 
  # Microstrain sensor node 
  # ****************************************************************** 
  microstrain_node = LifecycleNode(
    package    = _PACKAGE_NAME,
    executable = "microstrain_inertial_driver_node",
    name       = LaunchConfiguration('node_name'),
    namespace  = LaunchConfiguration('namespace'),
    parameters = [
      # Load the default params file manually, since this is a ROS params file, we will need to load the file manually
      yaml.safe_load(open(_DEFAULT_PARAMS_FILE, 'r')),

      # NOTE: These parameters are deprecated and will be removed in a future release. It is strongly recommended to use the new params_file option instead
      {
        "port"                  : LaunchConfiguration('port'),
        "aux_port"              : LaunchConfiguration('aux_port'),
        "baudrate"              : LaunchConfiguration('baudrate'),
        "imu_frame_id"          : LaunchConfiguration('imu_frame_id'),
        "gnss1_frame_id"        : LaunchConfiguration('gnss1_frame_id'),
        "gnss2_frame_id"        : LaunchConfiguration('gnss2_frame_id'),
        "filter_frame_id"       : LaunchConfiguration('filter_frame_id'),
        "filter_child_frame_id" : LaunchConfiguration('filter_child_frame_id'),
        "nmea_frame_id"         : LaunchConfiguration('nmea_frame_id'),
        "imu_data_rate"         : LaunchConfiguration('imu_data_rate'),
        "filter_data_rate"      : LaunchConfiguration('filter_data_rate'),
        "use_enu_frame"         : LaunchConfiguration('use_enu_frame'),
      },

      # If you want to override any settings in the params.yml file, make a new yaml file, and set the value via the params_file arg
      LaunchConfiguration('params_file'),

      # Supported overrides
      {
        "debug" : LaunchConfiguration('debug')
      },
    ]
  )

  # Optional configure and activate steps
  config_event = EmitEvent(
    event = ChangeState(
      lifecycle_node_matcher = matches_action(microstrain_node),
      transition_id          = Transition.TRANSITION_CONFIGURE
    ),
    condition = LaunchConfigurationEquals('configure', 'true')
  )
  activate_event = EmitEvent(
    event = ChangeState(
      lifecycle_node_matcher = matches_action(microstrain_node),
      transition_id          = Transition.TRANSITION_ACTIVATE
    ),
    condition = LaunchConfigurationEquals('activate', 'true')
  )

  launch_description.append(microstrain_node)
  launch_description.append(config_event)
  launch_description.append(activate_event)
  return LaunchDescription(launch_description)
  

 
 
