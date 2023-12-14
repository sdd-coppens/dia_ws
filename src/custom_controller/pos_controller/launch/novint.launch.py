from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="network",
            executable="receiver",
            name="receiver"
        ),
        Node(
            package="pos_controller",
            executable="new_novint",
            name="new_novint"
        ),
        # ExecuteProcess(cmd=["./home/stijn/thesis/OperatorDomain/build/operator_domain", "/home/stijn/thesis/OperatorDomain/simconfig.cfg"],
        #        output="screen"),
        ExecuteProcess(
            cmd=["/home/stijn/thesis/OperatorDomain/build/operator_domain", "../simconfig.cfg"],
            shell=True,
            output="screen",
        ),
    ])
