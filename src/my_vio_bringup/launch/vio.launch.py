"""
OpenVINS + 補正ノード + (RTAB-Map) + (foxglove_bridge) をまとめて起動する。

使い方:
  ros2 launch my_vio_bringup vio.launch.py                       # TUM VIの設定で起動
  ros2 launch my_vio_bringup vio.launch.py use_rtabmap:=true     # マッピングも
  ros2 launch my_vio_bringup vio.launch.py robot:=my_robot       # 自分のロボットの設定で
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import Node, SetParameter

def launch_setup(context):
    # ---- 引数 robot から、使う設定フォルダを決める ----
    robot = LaunchConfiguration('robot').perform(context)
    use_sim = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    bringup_dir = get_package_share_directory('my_vio_bringup')
    calib_dir = os.path.join(bringup_dir, 'config', robot)
    estimator_cfg = os.path.join(calib_dir, 'estimator_config.yaml')
    imucam_cfg = os.path.join(calib_dir, 'kalibr_imucam_chain.yaml')

    # ---- OpenVINS（自己位置推定） ----
    openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ov_msckf'), 'launch', 'subscribe.launch.py')),
        launch_arguments={'config_path': estimator_cfg}.items())

    # ---- 補正ノード（魚眼 -> 平行ステレオ） ----
    rectify = Node(
        package='my_vio_tools',
        executable='stereo_rectify',
        name='stereo_rectify',
        output='screen',
        parameters=[{
            'calib_file': imucam_cfg,
            'balance': 0.0,
            'publish_imu_to_cam_tf': False,  # imu->cam0 のTFが見つからないときは True に
        }])


    # ---- RTAB-Map（地図・ループ閉じ込み） ----
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')),
        launch_arguments={
            'use_sim_time': 'true' if use_sim else 'false',
            'stereo': 'true',
            'left_image_topic': '/stereo/left/image_rect',
            'right_image_topic': '/stereo/right/image_rect',
            'left_camera_info_topic': '/stereo/left/camera_info',
            'right_camera_info_topic': '/stereo/right/camera_info',
            'frame_id': 'imu',
            'visual_odometry': 'false',          # 自己位置はOpenVINSのものを使う
            'odom_topic': '/ov_msckf/odomimu',
            'approx_sync': 'true',
            'rtabmap_viz': 'false',              # 表示はLichtblickで行う
            'rtabmap_args':  '--delete_db_on_start --Grid/NoiseFilteringRadius 0.1 --Grid/RangeMax 3.0 --Grid/RayTracing true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_rtabmap')))

    return [SetParameter(name='use_sim_time', value=use_sim),
        openvins, rectify, rtabmap]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('robot', default_value='tum_vi',
                              description='config/ の下のフォルダ名'),
        DeclareLaunchArgument('use_rtabmap', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
