from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    share=Path(get_package_share_directory('liuo_uwb_bridge'))
    defaults={'port':'/dev/ttyUSB0','baud':'921600','config':str(share/'config/uwb.yaml'),
              'registry':str(share/'config/registry.yaml'),'namespace':'/uwb/n0','use_sim_time':'false'}
    declarations=[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
    cfg=LaunchConfiguration('config')
    sim=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return LaunchDescription(declarations+[
        Node(package='nlink_parser',executable='linktrack',namespace=LaunchConfiguration('namespace'),
             parameters=[cfg,{'port_name':LaunchConfiguration('port'),'baud_rate':ParameterValue(LaunchConfiguration('baud'),value_type=int),'use_sim_time':sim}],output='screen'),
        Node(package='liuo_uwb_bridge',executable='range_bridge',parameters=[cfg,{'registry_path':LaunchConfiguration('registry'),'input_namespace':LaunchConfiguration('namespace'),'use_sim_time':sim}],output='screen')])
