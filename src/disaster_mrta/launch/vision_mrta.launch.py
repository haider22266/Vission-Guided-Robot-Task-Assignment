"""Separate ROS processes for perception, allocation, controllers and monitoring."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def nodes(context):
    count=int(LaunchConfiguration('robot_count').perform(context))
    if not 1<=count<=5:
        raise ValueError('robot_count must be 1..5')
    common=[LaunchConfiguration('config_file').perform(context),{'robot_count':count}]
    overrides={name:LaunchConfiguration(name).perform(context) for name in
               ('allocator_backend','perception_backend','execution_backend','efficientnet_model','qmix_checkpoint')}
    result=[Node(package='disaster_mrta',executable=exe,output='screen',parameters=common+[overrides])
            for exe in ('image_perception','qmix_allocator','failure_monitor','evaluation_recorder')]
    result.extend(Node(package='disaster_mrta',executable='robot_task_controller',name=f'robot_{i+1}_controller',
                       output='screen',parameters=common+[overrides,{'robot_id':f'robot_{i+1}'}]) for i in range(count))
    supervisors=[RegisterEventHandler(OnProcessExit(target_action=node,
                 on_exit=[EmitEvent(event=Shutdown(reason='A required pipeline node exited'))])) for node in result]
    return result+supervisors


def generate_launch_description():
    default=str(Path(get_package_share_directory('disaster_mrta'))/'config'/'ros2_pipeline.yaml')
    defaults={'robot_count':'5','config_file':default,'allocator_backend':'qmix','perception_backend':'efficientnet',
              'execution_backend':'external','efficientnet_model':'','qmix_checkpoint':''}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=nodes)])
