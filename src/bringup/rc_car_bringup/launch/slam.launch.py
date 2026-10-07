"""
SLAM 진입점 (PC 측 실행) — 공통 입력 + ORB-SLAM3 + RViz2

    공통 입력   slam_inputs.launch.py      이미지 decompress
    SLAM       slam_orbslam3.launch.py    mono_node + scale_aligner
    뷰어        rviz.launch.py             config/slam.rviz

SLAM 계약:
    입력  /camera/image_decompressed, /camera/camera_info, /odometry/filtered,
          TF odom → base_link (Pi 의 EKF), static TF
    출력  TF map → odom
          odom → base_link 는 건드리지 않는다. ORB-SLAM3 가 추적을 잃어도 EKF 가
          위치를 계속 이어 주고, SLAM 은 map 기준 보정만 얹는다.

rosbag 으로 재현할 때:

    ros2 launch rc_car_bringup slam.launch.py use_sim_time:=true
    ros2 bag play <bag> --clock

사용법 (PC):
    ros2 launch rc_car_bringup slam.launch.py
    ros2 launch rc_car_bringup slam.launch.py mode:=localization
    ros2 launch rc_car_bringup slam.launch.py rviz:=false            # 뷰어 없이

ORB-SLAM3 인자(mode, settings_path, atlas_path, use_viewer, ...)는 그대로 넘기면
slam_orbslam3.launch.py 가 받는다.

RViz 만 따로 껐다 켜고 싶으면 rviz:=false 로 띄운 뒤
별도 터미널에서 `ros2 launch rc_car_bringup rviz.launch.py` 를 쓰면 된다.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = Path(get_package_share_directory("rc_car_bringup")) / "launch"

    inputs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "slam_inputs.launch.py"))
    )

    orbslam3_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "slam_orbslam3.launch.py"))
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_dir / "rviz.launch.py")),
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="RViz2 를 같이 띄운다 (config/slam.rviz)",
            ),
            inputs_launch,
            orbslam3_launch,
            rviz_launch,
        ]
    )
