from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node, SetParameter
import datetime

from launch.substitutions import LaunchConfiguration

def generate_launch_description():
 
    mission_circuit = Node(
            package='px4_fault_injection',
            namespace='px4_fault_injection',
            executable='mission_circuit.py',
            output='screen',
    )

    circuit_recorder = Node(
            package='px4_fault_injection',
            namespace='px4_fault_injection',
            executable='circuit_recorder.py',
            output='screen',
        )
    
    data_merger = Node(
            package='px4_fault_injection',
            namespace='px4_fault_injection',
            executable='data_merger_service.py',
            output='screen',
        )
    
    return LaunchDescription([
        circuit_recorder,
        mission_circuit,
        data_merger
    ])
