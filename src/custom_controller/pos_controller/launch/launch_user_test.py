from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="network",
            namespace="network1",
            executable="receiver",
            name="receiver"
        ),
        Node(
            package="network",
            namespace="network2",
            executable="sender",
            name="sender"
        ),
        # Node(
        #     package="pos_controller",
        #     namespace="pos_controller1",
        #     executable="fk_vector_predict.py",
        #     name="fk_vector_predict"
        # ),
        Node(
            package="pos_controller",
            namespace="pos_controller2",
            executable="delay_whiteboard_messages",
            name="delay_whiteboard_messages"
        ),
        Node(
            package="pos_controller",
            namespace="pos_controller3",
            executable="serial_node.py",
            name="serial_node"
        ),
        Node(
            package="pos_controller",
            namespace="pos_controller4",
            executable="udp_whiteboard_communicator",
            name="udp_whiteboard_communicator"
        ),
        # Node(
        #     package="pos_controller",
        #     namespace="pos_controller5",
        #     executable="refactor_novint",
        #     name="refactor_novint"
        # ),
        Node(
            package="pos_controller",
            namespace="pos_controller6",
            executable="ski_fk_two.py",
            name="ski_fk_two"
        )
    ])