from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='network',
            namespace='network1',
            executable='receiver',
            name='receiver'
        ),
        Node(
            package='network',
            namespace='network2',
            executable='sender',
            name='sender'
        ),
        Node(
            package='pos_controller',
            namespace='pos_controlelr1',
            executable='new_novint',
            name='new_novint'
        ),
        # ExecuteProcess(cmd=['/home/stijn/thesis/OperatorDomain/build/operator_domain', '/home/stijn/thesis/OperatorDomain/sim_config.cfg '],
        #        output='screen')
        # Node(
        #     package='turtlesim',
        #     namespace='turtlesim2',
        #     executable='turtlesim_node',
        #     name='sim'
        # ),
    ])