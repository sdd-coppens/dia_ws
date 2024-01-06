import launch
from launch.action import Action
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription
from launch_ros.actions import Node
import os
import shutil
import datetime

# Custom action to copy files and a folder
class CopyFilesAndFolderAction(Action):
    def __init__(self, csv_file_1_path, csv_file_2_path, csv_file_3_path, csv_file_4_path, folder_to_copy_path, folder_to_copy_path_calib, directory_name):
        super().__init__()  # Initialize the base class
        self.csv_file_1_path = csv_file_1_path
        self.csv_file_2_path = csv_file_2_path
        self.csv_file_3_path = csv_file_3_path
        self.csv_file_4_path = csv_file_4_path
        self.folder_to_copy_path = folder_to_copy_path
        self.folder_to_copy_path_calib = folder_to_copy_path_calib
        self.directory_name = directory_name

    def execute(self, context):
        shutil.copy(self.csv_file_1_path, os.path.join(self.directory_name, os.path.basename(self.csv_file_1_path)))
        shutil.copy(self.csv_file_2_path, os.path.join(self.directory_name, os.path.basename(self.csv_file_2_path)))
        shutil.copy(self.csv_file_3_path, os.path.join(self.directory_name, os.path.basename(self.csv_file_3_path)))
        shutil.copy(self.csv_file_4_path, os.path.join(self.directory_name, os.path.basename(self.csv_file_4_path)))
        shutil.copytree(self.folder_to_copy_path, os.path.join(self.directory_name, os.path.basename(self.folder_to_copy_path)))
        shutil.copytree(self.folder_to_copy_path_calib, os.path.join(self.directory_name, os.path.basename(self.folder_to_copy_path_calib)))
        return launch.ActionReturnValue()

def generate_launch_description():
    # Get current date and time
    current_datetime = datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")

    # Define the directory name
    directory_name = f"Experiment_Data/experiment_{current_datetime}"

    # Create the directory if it doesn't exist
    if not os.path.exists(directory_name):
        os.makedirs(directory_name)

    # Define the paths for the source CSV files and folder
    csv_file_1_path = '/dia_ws/imuData.csv'
    csv_file_2_path = '/dia_ws/log_file.csv'
    csv_file_3_path = '/dia_ws/calibImuData.csv'
    csv_file_4_path = '/dia_ws/frame_timestamps.csv'
    folder_to_copy_path = '/dia_ws/captured_frames'
    folder_to_copy_path_calib = '/dia_ws/calibration_frames'

    # Delete the original CSV files
    os.remove(csv_file_1_path)
    os.remove(csv_file_2_path)
    os.remove(csv_file_3_path)
    os.remove(csv_file_4_path)


    # Empty the original folder
    for filename in os.listdir(folder_to_copy_path):
        file_path = os.path.join(folder_to_copy_path, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f'Failed to delete {file_path}. Reason: {e}')

    # Empty the calibration folder
    for filename in os.listdir(folder_to_copy_path_calib):
        file_path = os.path.join(folder_to_copy_path_calib, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f'Failed to delete {file_path}. Reason: {e}')

    # Define the first two nodes: imu_recorder and cam_recorder
    imu_recorder_node = Node(
        package='experiment_nodes',
        executable='imu_recorder',
        name='imu_recorder'
    )

    cam_recorder_node = Node(
        package='experiment_nodes',
        executable='camera_recorder',
        name='camera_recorder'
    )

    # Define the third node: experiment_pose_seq
    experiment_pose_seq_node = Node(
        package='pos_controller',
        executable='experiment_pose_seq',
        name='experiment_pose_seq'
    )

    # Create a timer action for the delay
    timer_action = TimerAction(
        period=3.0,  # 3 seconds delay
        actions=[experiment_pose_seq_node]
    )

    # Handler for when imu_recorder_node exits
    imu_recorder_exit_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=imu_recorder_node,
            on_exit=[
                TimerAction(
                    period=5.0,  # Delay in seconds after imu_recorder_node exits
                    actions=[
                        CopyFilesAndFolderAction(csv_file_1_path, csv_file_2_path, csv_file_3_path, csv_file_4_path, folder_to_copy_path, folder_to_copy_path_calib, directory_name)
                    ]
                )
            ]
        )
    )

    # Define the launch description with all nodes and handlers
    return LaunchDescription([
        imu_recorder_node,
        cam_recorder_node,
        timer_action,  # Add the timer action with the third node
        imu_recorder_exit_handler  # Ensure this line ends with a comma if you add more actions below
    ])
