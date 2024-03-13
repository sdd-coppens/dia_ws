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
        # Node(
        #     package='pos_controller',
        #     namespace='pos_controller1',
        #     executable='fk_vector_predict.py',
        #     name='fk_vector_predict'
        # ),
        Node(
            package='pos_controller',
            namespace='pos_controller2',
            executable='delay_whiteboard_messages',
            name='delay_whiteboard_messages'
        ),

        # Node(
        #     package='pos_controller',
        #     namespace='pos_controller1',
        #     executable='operator_domain_manip',
        #     name='operator_domain_manip'
        # ),
        # Node(
        #     package='pos_controller',
        #     namespace='pos_controller2',
        #     executable='new_novint',
        #     name='new_novint',
        #     emulate_tty=True
        # ),
    ])