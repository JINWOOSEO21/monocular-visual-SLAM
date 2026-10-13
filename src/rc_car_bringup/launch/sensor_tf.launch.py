"""
Phase 4: 카메라-IMU 외부 캘리브레이션 (Static TF)

TF 프레임 구조:
    base_link
    ├── camera_link   (전방 카메라 — CSI imx219)
    │   └── camera        (optical frame)
    ├── c270_link     (Logitech C270 USB 웹캠)
    │   └── c270          (optical frame)
    └── imu_link      (MPU6050)

좌표계 규칙 (REP-103):
    x: 전방, y: 좌측, z: 상방

실측값 (단위: 미터, base_link 원점 = 뒷바퀴 축 중심):
    imu_link:    전방 0.060m, 우측 0.065m(→ y=-0.065), 상방 0.023m
    camera_link: 전방 0.205m, 우측 0.110m(→ y=-0.110), 상방 0.068m, 위로 30도 들림
    c270_link:   전방 0.185m, 좌측 0.002m(→ y=+0.002), 상방 0.076m

camera_link (2026-10-13 재장착 후 실측, IMU 축 기준):
    위치  (x=0.045, y=0.145, z=0.045)
    방향  원래 수평으로 달려 있던 카메라를 IMU x 축(= 로봇 좌우 축) 기준으로 30도 돌려
          위로 들었다. 광축이 IMU 축 기준 (0, cos30, sin30) 이다.

IMU 축은 아래 매핑을 따르므로 base_link 축으로는:

    위치  R_z(-pi/2) @ (0.045, 0.145, 0.045) = (0.145, -0.045, 0.045)
          + imu_link 원점 (0.060, -0.065, 0.023) = (0.205, -0.110, 0.068)
    방향  R_z(-pi/2) @ (0, cos30, sin30) = (cos30, 0, sin30)  → 전방에서 위로 30도
          ROS 의 pitch 는 양수가 머리를 숙이는 방향이라 pitch = -30도 (-0.5235988 rad)

광축 둘레의 회전(roll)은 방향 벡터로 정해지지 않는다. 이미지가 똑바로 나오므로 0 으로 둔다.

확인 필요 (2026-10-13):
    - y=-0.110 은 차 중심선에서 오른쪽 11cm 다. 이전 실측에서는 카메라가 IMU 의 왼쪽
      (IMU x=-0.06)이었으므로 IMU x 부호를 다시 확인할 것.
    - 재장착 직후 주행(17:06)의 키프레임 궤적에 평면을 맞추면 카메라 들림 각도가
      약 17.5도(좌우 약 5도)로 나온다. 30도와 12.5도 차이가 나는데, 재장착 전 카메라가
      실제로는 수평이 아니라 약 12도 아래를 보고 있었다면 설명된다. 주행 후
      `ros2 run orb_slam plot_map` 이 출력하는 "카메라 들림 각도" 로 다시 확인할 것.

이전 장착(재장착 전)의 camera_link 는 IMU 축 (-0.06, 0.10, 0.03) 실측에서 유도한
(0.160, -0.005, 0.053), roll/pitch/yaw 0 이었다.

c270_link 도 같은 방식으로 유도했다. IMU 축 기준 실측 (-0.067, 0.125, 0.053):

    R_z(-pi/2) @ (-0.067, 0.125, 0.053) = (0.125, 0.067, 0.053)
    (0.060, -0.065, 0.023) + (0.125, 0.067, 0.053) = (0.185, 0.002, 0.076)

같은 실측에서 CSI 카메라는 IMU 축 (0, 0.125, 0.05) 로 나왔고, 이는
base_link 로 (0.185, -0.065, 0.073) 에 해당한다. 즉 두 카메라는 전방
거리가 같고 y 로 0.067m(=6.7cm) 떨어진 나란한 배치다 — 나중에 스테레오로
쓸 때의 baseline 이 된다.

위 (0.185, -0.065, 0.073) 은 재장착 전의 CSI 측정이라 지금의 base_to_camera 와 다르다.

IMU 물리 장착 방향:
    IMU x축 → 로봇 우측(base_link -y)
    IMU y축 → 로봇 전방(base_link +x)
    IMU z축 → 로봇 상방(base_link +z)
    ∴ yaw = -π/2 회전 적용
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # base_link → camera_link
    # CSI 카메라는 전방에서 위로 30도 들려(pitch = -30도) 정방향으로 장착돼 있다. roll = 0.
    # 위치·각도 유도와 확인할 점은 파일 상단 docstring 참고.
    #
    # 한동안 roll=pi 가 들어가 있었다. 카메라를 뒤집어 달았던 시절의 값인데,
    # 그 뒤 장착이 바뀌어 지금은 이미지가 똑바로 나온다. 실측 (2026-09-15,
    # /csi_cam/image_raw 에서 받은 프레임): 사람 머리가 위, 침대가 아래로
    # 정방향이다. 같은 날 C270 프레임은 180도 뒤집혀 나왔고, 두 이미지를
    # 회전시켜가며 상관계수를 재면 서로 180도 관계다.
    #
    # TF 의 roll 은 "이미지를 돌리는 설정"이 아니라 "카메라 몸체가 어떻게
    # 달렸는지"를 기술하는 값이다. 정방향 이미지에 roll=pi 를 남겨두면 TF 가
    # 거짓을 말하게 되고, 삼각화된 3D 점의 상하좌우가 통째로 뒤집힌다:
    #
    #   roll=0  → 이미지 아래에 보이는 점이 base_link 에서 아래(-z)  ← 정방향 카메라
    #   roll=pi → 이미지 아래에 보이는 점이 base_link 에서 위 (+z)   ← 뒤집힌 카메라
    #
    # 정방향이 된 지금은 roll=0 이 맞다. 이미지 자체는 건드리지 않으므로
    # 캘리브레이션(intrinsics)은 그대로 유효하다.
    #
    # 뒤집힌 장착으로 되돌릴 일이 생기면 roll 을 pi 로 되돌리면 된다. 이미지를
    # 소프트웨어로 회전시키는 방식은 쓰지 않는다 — camera_ros 의 orientation 은
    # libcamera >= 0.2 를 요구하는데 이 Pi 는 0.1.0 이고, 별도 회전 노드는 Pi
    # CPU 와 지연을 늘리며 무엇보다 principal point 가 (W-1-cx, H-1-cy) 로
    # 바뀌어 캘리브레이션을 다시 잡아야 한다.
    base_to_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_camera_tf",
        arguments=[
            "--x",
            "0.205",  # 전방 20.5cm
            "--y",
            "-0.110",  # 우측 11.0cm (IMU x 부호 확인 필요 — docstring 참고)
            "--z",
            "0.068",  # 상방 6.8cm
            "--roll",
            "0.0",  # 정방향 장착
            "--pitch",
            "-0.5235988",  # -30도: 광축이 전방에서 위로 30도
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "camera_link",
        ],
    )

    # camera_link → camera (optical frame)
    # ROS 카메라 광학 프레임 규칙: z 전방, x 우측, y 하방
    # camera_link(x전방,y좌,z상) → optical(z전방,x우,y하)
    # 회전: roll=-π/2, yaw=-π/2
    # child-frame-id 는 camera_ros 노드가 이미지 헤더에 박는 frame_id ('camera') 와 일치시켜야
    # SLAM 이 TF lookup 가능. (기존 'camera_optical_frame' → 'camera' 로 변경)
    camera_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_to_optical_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.5707963",  # -π/2
            "--pitch",
            "0.0",
            "--yaw",
            "-1.5707963",  # -π/2
            "--frame-id",
            "camera_link",
            "--child-frame-id",
            "camera",
        ],
    )

    # base_link → c270_link  (Logitech C270 USB 웹캠)
    #
    # 아직 이 프레임을 쓰는 노드는 없다. C270 은 UVC 장치라 camera_ros(libcamera)
    # 가 아니라 usb_cam 같은 별도 노드가 필요하고, 지금 단안 SLAM 은 CSI 카메라만
    # 쓴다. 나중에 스테레오로 넘어갈 때 바로 쓸 수 있도록 외부 파라미터만 먼저
    # 기록해 둔다. 구독자가 없으므로 단안 파이프라인 동작에는 영향이 없다.
    #
    # 좌표 유도는 파일 상단 docstring 참고 (IMU 축 실측 -0.067, 0.125, 0.053).
    #
    # 회전: CSI 와 마찬가지로 roll=π. C270 도 상하 반전으로 장착돼 있다.
    #
    # 처음에는 정방향 장착으로 가정해 roll=0 을 넣었는데, 두 카메라를 서로 다른
    # 노드 이름으로 동시에 띄워(토픽 충돌 없이) 같은 순간의 프레임을 한 장씩 받아
    # 비교해보니 틀렸다. CSI 기준으로 C270 을 회전시켜가며 상관계수를 잰 결과:
    #
    #     0도: -0.5620    90도: -0.2341    180도: +0.5717    270도: +0.1864
    #
    # 180도가 뚜렷하게 최고고 정방향은 음의 상관이다. 두 카메라가 6.7cm 떨어져
    # 있어 절대값이 1 에 못 미치는 것은 시차 때문이며, 각도 판별에는 영향이 없다.
    # (90/270 은 종횡비가 달라 리사이즈 후 비교한 참고값이다.)
    base_to_c270 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_c270_tf",
        arguments=[
            "--x",
            "0.185",  # 전방 18.5cm
            "--y",
            "0.002",  # 좌측 0.2cm (거의 중심선)
            "--z",
            "0.076",  # 상방 7.6cm
            "--roll",
            "3.1415927",  # π : 상하 반전 장착 (CSI 와 동일)
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "c270_link",
        ],
    )

    # c270_link → c270 (optical frame)
    # 규칙은 camera_to_optical 과 동일 (z 전방, x 우측, y 하방).
    # child-frame-id 는 나중에 C270 을 띄우는 노드가 이미지 헤더에 박는
    # frame_id 와 일치시켜야 한다 (usb_cam 이면 camera_frame_id 파라미터).
    c270_to_optical = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="c270_to_optical_tf",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "-1.5707963",  # -π/2
            "--pitch",
            "0.0",
            "--yaw",
            "-1.5707963",  # -π/2
            "--frame-id",
            "c270_link",
            "--child-frame-id",
            "c270",
        ],
    )

    # base_link → imu_link
    # IMU x→로봇우측(-y), y→로봇전방(+x), z→로봇상방(+z)  ∴ yaw=-π/2
    base_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_imu_tf",
        arguments=[
            "--x",
            "0.060",  # 전방 6.0cm
            "--y",
            "-0.065",  # 우측 6.5cm
            "--z",
            "0.023",  # 상방 2.3cm
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "-1.5707963",  # -π/2 : IMU y축이 로봇 전방
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "imu_link",
        ],
    )

    return LaunchDescription(
        [
            base_to_camera,
            camera_to_optical,
            base_to_c270,
            c270_to_optical,
            base_to_imu,
        ]
    )
