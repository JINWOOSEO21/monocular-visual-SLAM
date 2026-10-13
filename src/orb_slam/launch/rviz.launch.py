"""
RViz2 로 SLAM 상태를 실시간으로 보는 launch (PC 측 실행)

slam.launch.py 와 별개로 띄울 수 있다. RViz 만 껐다 켜도 SLAM 은 계속 돌기 때문에,
같은 launch 에 묶는 것보다 이쪽이 실사용에 편하다.

표시되는 것:
    ORB Map Points /orb_slam3/map_points_map  scale_aligner 가 map 프레임·미터로 옮긴 맵 포인트
                   (원본 /orb_slam3/map_points 는 orb_world 프레임이라 TF 로 map 에 닿지 않는다.
                    스케일이 확정된 뒤부터 나온다.)
    Odometry (EKF) /odometry/filtered     EKF 추정 위치
    Odometry (ORB) /orb_slam3/odometry    scale_aligner 가 미터화한 SLAM 위치
    Camera         /camera/image_decompressed

Fixed Frame 은 map 이다. scale_aligner 가 map→odom 을 아직 발행하기 전이면
"No transform from [odom] to [map]" 경고가 잠깐 뜨는데, ORB-SLAM3 가 초기화되고
스케일이 잡히면 사라진다.

사용법 (PC):
    ros2 launch orb_slam rviz.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("orb_slam")) / "config" / "slam.rviz"
    )

    rviz_config = LaunchConfiguration("rviz_config")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_config,
                description="RViz2 config file path",
            ),
            rviz_node,
        ]
    )
