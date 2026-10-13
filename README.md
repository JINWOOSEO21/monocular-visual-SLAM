# Monocular Visual SLAM (ORB-SLAM3) for an RC Car

라즈베리파이 4B 기반 차동구동(differential drive) RC카의 ROS 2 워크스페이스.
단안 카메라 + 휠 엔코더 + IMU를 EKF로 융합해 오도메트리를 만들고,
PC 쪽에서 ORB-SLAM3 단안 SLAM을 돌린 뒤 휠 오도메트리로 미터 스케일을 맞춘다.

무거운 SLAM 연산은 PC가 맡고 Pi는 센서 퍼블리시와 모터 제어만 담당하는
분산 구조다. 둘은 같은 Wi-Fi의 ROS 2 DDS로 통신한다.

```
┌─────────────────── Raspberry Pi 4B ────────────────────┐      ┌──────── PC ────────┐
│                                                        │      │                    │
│  IMX219 CSI ──▶ camera_ros ──▶ /camera/image_raw       │      │                    │
│                             └▶ /camera/image_raw/      │─Wi-Fi▶  image_transport   │
│                                        compressed      │      │     republish      │
│                                                        │      │         │          │
│  MPU6050 ──▶ mpu6050_node ──▶ /imu/data_raw            │      │         ▼          │
│   (I2C)             │                                  │      │  orb_slam3 (mono)  │
│                     ▼                                  │      │         │          │
│              imu_kalman_node ──▶ /imu/data             │      │   scale_aligner    │
│                                    │                   │      │         ▼          │
│  엔코더 ──▶ encoder_node ──▶ /encoder/ticks            │      │   TF: map → odom   │
│  (GPIO)            │                                   │      │ /orb_slam3/odometry│
│                    ▼                                   │      └────────────────────┘
│             odometry_node ──▶ /odom                    │
│                                    │                   │
│                    ┌───────────────┴────┐              │
│                    ▼                    ▼              │
│               robot_localization EKF ──▶ /odometry/filtered
│                                          TF: odom → base_link
│                                                        │
│  /cmd_vel ──▶ pid_controller_node ──▶ L298N ──▶ 모터   │
│               (또는 motor_driver_node)                 │
└────────────────────────────────────────────────────────┘
```

## 하드웨어

| 항목 | 사양 |
|---|---|
| 보드 | Raspberry Pi 4B 4GB, Ubuntu 22.04, ROS 2 Humble |
| 카메라 | IMX219 (Arducam), CSI 연결, libcamera + 하드웨어 ISP. **정방향 장착, 전방에서 위로 20° 들림** → TF `base_link→camera_link` 는 `roll=0`, `pitch=-20°` |
| 카메라 (보조) | Logitech C270, USB(UVC). **상하 반전 장착** → TF `base_link→c270_link` 에 `roll=π`. 아직 구독하는 노드 없음 |
| IMU | MPU6050 (GY-521), I2C bus 1 @ `0x68` |
| 모터 드라이버 | L298N |
| 엔코더 | 단채널 광학 엔코더 ×2 (방향 정보 없음, 펄스만 카운트) |
| 휠 | 직경 65 mm, 휠베이스 150 mm, 40 펄스/회전 (BOTH 엣지) |

### GPIO 핀맵 (BCM)

| 기능 | 핀 |
|---|---|
| 좌 모터 ENA / IN1 / IN2 | GPIO24 / GPIO22 / GPIO23 |
| 우 모터 ENB / IN3 / IN4 | GPIO25 / GPIO27 / GPIO26 |
| 좌 엔코더 | GPIO17 |
| 우 엔코더 | GPIO16 |

> L298N IN 핀은 좌·우 모두 코드 기준과 반대로 배선되어 있다.
> 그래서 `left_invert` / `right_invert` 가 둘 다 `true` 로 설정돼 있다.

### TF 프레임

```
map ──(scale_aligner)──▶ odom ──(EKF)──▶ base_link ──┬──▶ camera_link ──▶ camera
                                               └──▶ imu_link
```

`base_link` 원점은 뒷바퀴 축 중심. 오프셋은 `rc_car_bringup/launch/sensor_tf.launch.py` 참고.
IMU는 x축이 로봇 우측, y축이 전방을 향하도록 장착되어 있어 `yaw = -π/2` 가 걸려 있다.

## 디렉터리 구조

```
orb-slam/
├── README.md
├── pyproject.toml              # ruff 설정
├── scripts/
│   └── build_third_party.sh    # ORB-SLAM3 + Pangolin 빌드 (PC 전용, colcon 이전에 1회)
├── src/
│   ├── imu_mpu6050/            # [Pi] MPU6050 IMU 드라이버 + 칼만 필터
│   ├── motor_encoder/          # [Pi] 휠 엔코더 카운트 + 휠 오도메트리
│   ├── pid_motor_controller/   # [Pi] closed-loop 모터 제어 (권장)
│   ├── motor_controller/       # [Pi] open-loop 모터 제어 (둘 중 하나만 실행)
│   ├── rc_car_bringup/         # [Pi] 센서 스택 · static TF · EKF · 카메라 캘리브레이션
│   └── orb_slam/               # [PC] ORB-SLAM3 래퍼 + SLAM launch · RViz 설정
└── third_party/                # COLCON_IGNORE
    ├── ORB_SLAM3/              #   upstream 소스 복사 + 로컬 패치
    └── Pangolin/               #   git submodule (v0.6)
```

Pi 와 PC 는 같은 저장소 · 같은 브랜치를 쓴다. `orb_slam` 은 ORB-SLAM3 라이브러리가
없으면 CMake 단계에서 스스로 빠지므로, Pi 에서 `colcon build` 를 해도 SLAM 쪽은
빌드되지 않는다.

## 패키지

### `imu_mpu6050`
MPU6050을 I2C로 직접 읽어 IMU 데이터를 퍼블리시한다.

- **`mpu6050_node`** → `/imu/data_raw` (50 Hz), `/imu/temperature`
  - 기동 시 정지 상태에서 자이로 바이어스를 매번 다시 추정한다 (온도에 따라 변하므로).
  - 가속도계는 `config/imu_calibration.yaml` 의 계수로 보정한다:
    `a_corr = (a_raw - bias) * scale`
  - 기동 시 정지 상태 `|a|` 를 9.807 m/s² 와 비교하는 헬스체크를 출력한다.
- **`imu_kalman_node`** → `/imu/data`
  - 2-state 칼만 필터(각도 + 자이로 바이어스)로 roll/pitch를 추정해 orientation을 채운다.
  - 지자기 센서가 없어 yaw는 자이로 적분이라 드리프트한다. EKF가 `differential` 모드로 받아 변화량만 쓴다.
- **`scripts/calibrate_accel.py`** — 가속도계 6면 캘리브레이션 ([아래](#가속도계-캘리브레이션) 참고)

### `motor_encoder`
- **`encoder_node`** → `/encoder/ticks` (`Int32MultiArray [left, right]`, 50 Hz)
  - GPIO 인터럽트로 양쪽 엣지를 센다. **부호 없는 누적 카운트**만 퍼블리시한다.
- **`odometry_node`** → `/odom`
  - 차동구동 기구학으로 위치를 적분한다.
  - 엔코더에 방향 정보가 없어서 `/cmd_vel` 의 부호로 각 바퀴의 진행 방향을 추론한다.
    즉 **명령과 반대로 밀리면(슬립·외력) 오도메트리가 그 방향을 잘못 안다.**
  - TF는 EKF가 broadcast하므로 기본적으로 `publish_tf: false`.

### `pid_motor_controller` (권장 — closed-loop)
- **`pid_controller_node`** — `/cmd_vel` + `/encoder/ticks` → PID → L298N PWM
  - 피드포워드(측정한 duty–속도 1차 모델) + PID 보정 구조.
  - 방향이 바뀌면 적분항을 리셋한다.
  - `/cmd_vel` 이 `cmd_timeout`(0.5 s) 동안 안 오면 정지 — Wi-Fi 끊김 대비.
- **`step_cmd_vel_node`** — 스텝 응답 시험용 `/cmd_vel` 퍼블리셔
- **`analyze_pid_step_response`** — 컨트롤러가 남긴 CSV 로그에서 상승시간·오버슈트 등 지표 계산

### `motor_controller` (대안 — open-loop)
- **`motor_driver_node`** — `/cmd_vel` 을 PWM duty로 직접 매핑. 엔코더 피드백 없음.

> ⚠️ **`pid_controller_node` 와 `motor_driver_node` 를 동시에 실행하면 안 된다.**
> 같은 GPIO 핀(ENA/ENB/IN1–4)을 제어하므로 둘 중 하나만 골라 쓴다.

### `rc_car_bringup` (Pi)
Pi 쪽 센서 스택 런치와 설정 모음.

| 런치 | 실행 위치 | 내용 |
|---|---|---|
| `sensors.launch.py` | Pi | 카메라 + IMU + 엔코더/오도메트리 + static TF + EKF (모터 제외) |
| `ekf.launch.py` | Pi | `robot_localization` EKF 단독 (`config/ekf.yaml`) |
| `sensor_tf.launch.py` | Pi | `base_link` → `camera_link` / `camera` / `imu_link` static TF |
| `intrinsic_calibration.launch.py` | Pi + PC | 카메라 내부 파라미터 캘리브레이션 ([아래](#카메라-내부-파라미터) 참고) |

> `intrinsic_calibration.launch.py` 는 카메라 노드를 Pi 에서 띄워야 하므로
> (`mode:=camera`) Pi 에서도 빌드되는 이 패키지에 둔다.

### `orb_slam` (PC 전용)
ORB-SLAM3 단안 래퍼와 SLAM 실행에 필요한 런치 · 설정 전부.

| 런치 | 내용 |
|---|---|
| `slam.launch.py` | SLAM 진입점: 아래 세 개를 묶어 실행 |
| `slam_inputs.launch.py` | 이미지 decompress (`/camera/image_raw/compressed` → `/camera/image_decompressed`) |
| `slam_orbslam3.launch.py` | `mono_node` + `scale_aligner` |
| `rviz.launch.py` | RViz2 단독 (`config/slam.rviz`) |

- **`mono_node`** — `/camera/image_decompressed` → `System::TrackMonocular`
  → `/orb_slam3/camera_pose`, `/orb_slam3/state`, `/orb_slam3/map_points`, 서비스 `~/save_map`
- **`scale_aligner`** — 스케일 없는 단안 포즈를 `/odometry/filtered` 로 미터화해
  `/orb_slam3/odometry` 와 TF `map → odom` 을 발행한다.
- `third_party/ORB_SLAM3/lib/libORB_SLAM3.so` 가 없으면 빌드에서 자동으로 빠진다 (Pi).

## 빌드

### PC — third_party (ORB-SLAM3 + Pangolin)

| 경로 | 출처 | 형태 |
|---|---|---|
| `third_party/ORB_SLAM3` | [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) `4452a3c` | 소스 복사(vendor) + 로컬 패치 |
| `third_party/Pangolin` | [stevenlovegrove/Pangolin](https://github.com/stevenlovegrove/Pangolin) `v0.6` | git submodule (수정 없음) |

PC 에서 clone 할 때는 submodule 까지 받는다 (빠뜨려도 빌드 스크립트가 받아 온다).
Pi 는 Pangolin 이 필요 없으므로 submodule 을 받지 않아도 된다.

```bash
git clone --recursive git@github.com:JINWOOSEO21/monocular-visual-SLAM.git   # PC
git submodule update --init third_party/Pangolin                           # 이미 clone 했다면
```

ORB-SLAM3 는 git 히스토리에 **원본 커밋**(upstream `4452a3c` 그대로, 데이터셋 예제·
`evaluation/` 제외)과 그 위의 **패치 커밋**이 따로 있다. 무엇을 고쳤는지는 패치 커밋
하나로 볼 수 있고, 소스 안의 변경 지점에는 `rc_car patch:` 주석이 달려 있다:

```bash
git log --oneline -- third_party/ORB_SLAM3                  # 원본 / 패치 커밋 확인
git show <패치 커밋> -- third_party/ORB_SLAM3                 # 변경 내용
git format-patch -1 <패치 커밋> -- third_party/ORB_SLAM3      # 다른 버전에 다시 적용할 .patch 추출
```

패치 요약:

- `System::Shutdown()` 이 뷰어를 멈추고 매핑/루프 스레드를 기다린 뒤 Atlas 를 저장
  (원본은 Ctrl-C 시 segfault/hang)
- `Map::PreSave` / `MapPoint::PreSave` 가 컨테이너 복사본을 순회 (원본은 Atlas 저장 시 SIGSEGV)
- `System::GetCurrentMap()` 추가 — `mono_node` 가 맵 병합 시 스케일 epoch 를 새로 시작하는 데 쓴다
- ORB_SLAM3 / g2o / DBoW2 CMakeLists 에서 `-march=native` 제거
  (있으면 "New Map created" 직후 `double free or corruption` 으로 죽는다)

```bash
./scripts/build_third_party.sh     # Pangolin → DBoW2 → g2o → ORB_SLAM3, vocabulary 압축 해제
```

ORB_SLAM3 빌드 트리에 `make clean` / `--clean-first` 를 쓰지 말 것 — g2o 라이브러리까지 지운다.
vocabulary 는 `~/.ros/orbslam3/ORBvoc.txt` 로 풀린다.

### 워크스페이스

```bash
cd ~/workspace/codes/orb-slam     # Pi 에서는 ~/orb-slam
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 을 쓰면 Python 파일과 config YAML을 고쳐도 재빌드가 필요 없다.
(`setup.py` / `package.xml` / 런치 파일을 바꿨을 때는 다시 빌드해야 한다.)
`third_party/` 는 `COLCON_IGNORE` 로 colcon 대상에서 빠져 있다.

> 패키지 폴더를 옮긴 뒤(예: `src/` 재구조화)에는 `build/` · `install/` 에 남은
> symlink 가 예전 소스 경로를 가리킨다. 한 번 지우고 다시 빌드한다:
> `rm -rf build install log && colcon build --symlink-install`

### 의존 패키지

```bash
sudo apt install ros-humble-camera-ros ros-humble-robot-localization \
                 ros-humble-image-transport
```

Python: `RPi.GPIO`, `smbus`, `PyYAML`

## 실행

### 1. Pi — 센서 스택

```bash
ros2 launch rc_car_bringup sensors.launch.py      # 카메라 15 Hz / JPEG q80
```

### 2. Pi — 모터 제어 (별도 터미널, 둘 중 하나만)

```bash
ros2 launch pid_motor_controller pid_controller.launch.py   # closed-loop (권장)
# 또는
ros2 launch motor_controller motor.launch.py                 # open-loop
```

### 3. PC — SLAM + 시각화

```bash
ros2 launch orb_slam slam.launch.py
```

이 하나로 공통 입력(이미지 decompress) + ORB-SLAM3 + RViz2 가 같이 뜬다.

```
Pi: 센서 + EKF ──▶ /camera/image_raw/compressed, /odometry/filtered, TF odom→base_link
                          │
PC: slam_inputs ──▶ /camera/image_decompressed
                          │
    mono_node ──▶ /orb_slam3/camera_pose (스케일 없음)
                          │
    scale_aligner ──▶ /orb_slam3/odometry, TF map→odom
```

SLAM 은 `odom→base_link`(EKF)를 건드리지 않고 `map→odom` 만 발행한다.
ORB-SLAM3 가 추적을 잃어도 EKF 가 위치를 계속 이어 준다.
rosbag 으로 재현할 때는 `use_sim_time:=true` + `ros2 bag play <bag> --clock`.

| 인자 | 기본 | 설명 |
|---|---|---|
| `rviz` | `true` | RViz2 (`config/slam.rviz`) |
| `use_sim_time` | `false` | rosbag 재생 시 `true` |
| `mode` | `slam` | `slam` / `localization` |
| `vocabulary` | `~/.ros/orbslam3/ORBvoc.txt` | BoW vocabulary |
| `settings_path` | (패키지 `config/imx219_820x616.yaml`) | 카메라/ORB 설정 |
| `atlas_path` | `~/.ros/orbslam3/atlas` | Atlas 저장/불러오기 경로 |
| `use_viewer` | `false` | ORB-SLAM3 자체 Pangolin 뷰어 |
| `trajectory_path` | `~/.ros/orbslam3/kf_trajectory.txt` | 종료 시 키프레임 궤적(TUM) |
| `map_points_path` | `~/.ros/orbslam3/map_points.ply` | 종료 시 맵 포인트(ASCII PLY: `x y z observations t_first`) |

```bash
ros2 launch orb_slam slam.launch.py rviz:=false          # 뷰어 없이 (헤드리스)
ros2 launch orb_slam slam.launch.py use_viewer:=true     # 특징점/키프레임 내부까지
```

Wi-Fi 대역폭 때문에 raw 이미지는 17 fps까지 떨어져서, Pi는 compressed만 보내고
PC가 `image_transport/republish` 로 풀어서 ORB-SLAM3 에 넣는다.

루프 클로저는 같은 장소를 **같은 방향으로** 다시 지날 때만 일어난다.
테스트 코스는 한 방향으로 도는 루프로 잡는다.

### 4. RViz2 에 보이는 것

위 3번에서 RViz 가 이미 같이 떴다. SLAM 을 계속 돌린 채 RViz 만 껐다 켜고 싶으면
`rviz:=false` 로 띄운 뒤 별도 터미널에서:

```bash
ros2 launch orb_slam rviz.launch.py
```

| Display | 토픽 | 보이는 것 |
|---|---|---|
| ORB Map Points | `/orb_slam3/map_points_map` | ORB-SLAM3 맵 포인트 (`scale_aligner` 가 map 프레임·미터로 변환, 스케일 확정 후부터) |
| Odometry (EKF) | `/odometry/filtered` | EKF 추정 위치 |
| Odometry (ORB-SLAM3) | `/orb_slam3/odometry` | 미터화된 SLAM 위치 |
| Camera | `/camera/image_decompressed` | 카메라 원본 |

Fixed Frame 은 `map` 이다. ORB-SLAM3 가 초기화되고 스케일이 잡히기 전에는
`No transform from [odom] to [map]` 경고가 뜨는데 정상이다.

> 다른 설정 파일을 쓰려면 `rviz_config:=/경로/my.rviz` 로 넘긴다.
> 이 인자는 `slam.launch.py` 에서도 그대로 받는다.

### 5. 주행 후 궤적·맵 포인트 그리기

SLAM 을 Ctrl-C 로 끄면 `trajectory_path` / `map_points_path` 에 **종료 시점의 현재 맵**의
키프레임 궤적과 맵 포인트(삼각측량된 ORB 특징점의 3D 위치)가 저장된다. 둘 다 `orb_world`
프레임(첫 키프레임의 카메라 광학 프레임), 스케일 없는 ORB 단위다.

```bash
ros2 run orb_slam plot_map --scale 2.39                    # 스케일 = scale_aligner 의 "스케일 확정" 값
ros2 run orb_slam plot_map --scale 2.39 --out map.png      # 창 대신 파일로
ros2 run orb_slam plot_map --level mount --pitch-deg 20    # 장착 각도(TF pitch 와 같은 값)로 수평 정렬

# 마무리용 결과물 세 가지 (정적 그림, 주행 애니메이션, 3D 뷰어)
ros2 run orb_slam plot_map --scale 2.6 --name test_1820 \
    --events ~/.ros/log/<그 실행>/launch.log \
    --out map.png --animate run.gif --html viewer.html
```

`--events` 를 주면 그 실행의 launch.log 에서 새 맵 / 맵 병합 / 루프 클로저 시각을 읽어
그림·애니메이션·뷰어에 표시한다 (실시간 실행 기준). `--html` 은 Plotly 로 그리는 자체
완결 페이지라 브라우저로 바로 연다. `kf_trajectory.txt` / `map_points.ply` 는 다음 실행이
덮어쓰므로, 남길 실행은 파일을 따로 복사해 두고 `--trajectory` / `--points` 로 넘긴다.
저장된 궤적은 루프 클로저까지 끝난 최종 지도라 애니메이션은 최종 지도 위의 재생이다
(주행 중 드리프트가 보정되는 모습은 rosbag 으로 `/orb_slam3/odometry` 를 남겨야 보인다).

애니메이션과 3D 뷰어는 맵 포인트를 **처음 관측된 시각**에 하나씩 나타나게 해서 지도가
쌓여 가는 모습을 보여 주고, 방금 관측된 점은 주황색으로 잠깐 강조한다. 그 시각은
`map_points.ply` 의 `t_first`(그 점을 관측한 가장 이른 키프레임의 타임스탬프, 궤적 파일과 같은
시간축)를 쓴다. `t_first` 가 없는 예전 PLY 는 키프레임 포즈와 카메라 내부 파라미터로 점이
시야에 처음 들어온 시각을 추정한다 (가림은 고려하지 않는다). 처음부터 모든 점을 보이려면
`--static-points`, 뷰어에서는 "관측된 점만 쌓기" 체크를 끈다.

카메라가 기울어 있으면 궤적도 기울어 보이므로 바닥을 수평으로 돌려서 그린다. 기본
`--level pca` 는 궤적에 평면을 맞추는 방식이라 장착 각도를 몰라도 된다. 출력되는
"궤적 평면으로 추정한 카메라 들림 각도" 는 `sensor_tf.launch.py` 의 카메라 pitch 를
검증하는 데 쓸 수 있다. 정렬 후 키프레임 높이 표준편차가 0 에 가까워야 정상이다.

## 캘리브레이션

### 가속도계 캘리브레이션

MPU6050(특히 저가 GY-521 클론)은 가속도 감도 오차가 10%를 넘는 개체가 흔하다.
이 저장소의 개체는 정지 상태에서 `|a| = 11.16 m/s²` (정상 9.807, **+13.9%**)를 읽는다.
`ACCEL_CONFIG` 레지스터는 정상이므로 설정 문제가 아니라 센서 개체 오차다.

평평하고 단단한 책상 위에서:

```bash
python3 src/imu_mpu6050/scripts/calibrate_accel.py
```

6개 자세(X위/X아래/Y위/Y아래/Z위/Z아래)를 안내대로 잡아주면 축별 bias와 scale을 푼다:

```
bias  = (r_up + r_dn) / 2
scale = 2g / (r_up - r_dn)
```

결과는 `src/imu_mpu6050/config/imu_calibration.yaml` 에 저장되고,
다음 `mpu6050_node` 기동 때 자동으로 적용된다. 로그에 이렇게 뜨면 정상이다:

```
[INFO] [mpu6050_node]: Accel correction: bias=[...], scale=[...]
[INFO] [mpu6050_node]: Gravity check OK: |a|=9.8xx m/s²
```

보정을 안 했으면 `Gravity check FAILED` 경고가 뜬다.

> **참고:** 현재 EKF는 가속도(`ax/ay/az`)를 융합에서 제외하고 있어서(`ekf.yaml`의
> `imu0_config`) 이 오차가 위치 추정에 직접 새어 들어가지는 않는다. 다만
> `imu_kalman_node` 가 중력벡터로 roll/pitch를 계산하므로 **축별 오프셋**은
> 자세 편향(약 2–4°)으로 나타난다. 균일 스케일 오차는 `atan2` 비율에서 상쇄된다.

### 카메라 내부 파라미터

카메라마다 따로 잡아야 한다. `camera:=` 로 고른다 (기본값 `csi`).

```bash
ros2 launch rc_car_bringup intrinsic_calibration.launch.py                # CSI (imx219)
ros2 launch rc_car_bringup intrinsic_calibration.launch.py camera:=usb    # USB 웹캠 (C270)
```

7×5 체커보드(사각형 25 mm) 기준. 결과는 `~/.ros/camera_info/` 에 저장되며,
파일명에 카메라 id 와 해상도가 들어가므로 두 카메라의 결과는 저절로 분리된다.

| 카메라 | 포맷 | 결과 파일 |
|---|---|---|
| `csi` | RGB888 | `imx219__base_soc_i2c0mux_i2c_1_imx219_10_820x616.yaml` |
| `usb` | YUYV | `C270HDWEBCAM__base_scb_pcie_..._046d_0825_820x616.yaml` |

> 포맷이 다른 이유: C270 은 libcamera 기준 MJPEG/YUYV 만 지원하고 RGB888 을
> 내보내지 못한다. CSI 와 같은 설정으로 띄우면 `unsupported pixel format
> "RGB888"` 로 죽는다.

> **USB 웹캠을 다른 포트에 꽂으면 id 가 바뀐다.** libcamera 가 붙이는 id 에 USB
> 포트 경로(`usb@0,0-1.2`)가 들어 있고 `camera_ros` 는 완전히 같은 문자열만
> 받는다. 그럴 때는 `camera_id:=<id>` 로 직접 넘기면 된다. 현재 id 목록은
> `camera` 파라미터 없이 `camera_node` 를 띄우면 기동 로그의 `>> cameras:` 에
> 나온다.

> **카메라를 다시 장착하면 반드시 다시 잡아야 한다.** 장착이 바뀌면 TF 의 `roll`
> 도 같이 맞춰야 한다 (정방향이면 0, 상하 반전이면 π).
>
> 화면에 보이는 대로 진행하면 된다. C270 처럼 반전 장착이라 체커보드가 뒤집혀
> 보여도 그대로 잡는다 — `cameracalibrator` 는 체커보드 방향을 가리지 않고,
> 내부 파라미터는 실제로 퍼블리시되는 이미지 기준으로 나와야 런타임과 일치한다.
> 이미지를 돌려서 잡으면 principal point 가 `(W-1-cx, H-1-cy)` 로 어긋난다.
>
> 이전 값들은 `~/.ros/camera_info/*.yaml.*.bak` 으로 백업해 두었다.

### 엔코더 / 하드웨어 점검

`~/hw_check/` 에 배선 점검용 스탠드얼론 스크립트가 있다 (저장소 밖).

```bash
python3 ~/hw_check/test_encoder_manual.py   # 바퀴를 손으로 돌리며 틱 확인
python3 ~/hw_check/test_imu_motion.py       # IMU를 기울이며 축별 반응 확인
```

## 센서 융합 설정

`rc_car_bringup/config/ekf.yaml` (`two_d_mode: true`):

| 소스 | 사용하는 상태 | 이유 |
|---|---|---|
| `/odom` (휠) | `vx`, `vy`, `vyaw` | pose는 슬립이 누적되므로 twist만 사용. `vy=true` 는 비홀로노믹 제약(옆방향 속도 0) |
| `/imu/data` | `yaw`, `vyaw` (differential) | 절대 방위 기준이 없으므로 변화량만 사용 |
| 가속도 | **사용 안 함** | RC카 진동으로 노이즈가 크고, 이중적분은 드리프트가 심함 |

## 알려진 이슈

- **엔코더에 방향 정보가 없다.** 단채널이라 `/cmd_vel` 부호로 방향을 추론한다.
  외력이나 슬립으로 명령과 반대로 움직이면 오도메트리가 틀린다. 정확도가 더 필요하면
  쿼드러처(2채널) 엔코더로 바꿔야 한다.
- **yaw 드리프트.** 지자기 센서가 없어 yaw는 자이로 적분이다. ORB-SLAM3 의 루프
  클로저가 `map → odom` 에서 보정하지만, 특징점이 부족한 구간에서는 누적된다.
- **문서와 코드 불일치.** 일부 주석과 `rc_car_bringup/package.xml` 이 orientation
  필터로 `imu_filter_madgwick` 을 언급하지만, 실제로 실행되는 것은
  `imu_mpu6050/imu_kalman_node` 다.
- **`/dev/media*` 번호는 재부팅마다 바뀐다.** libcamera가 알아서 찾으므로 보통
  문제되지 않지만, `media-ctl` 을 직접 쓸 때는 `unicam` + `imx219` 문자열로 탐색해야 한다.
- **USB 웹캠을 꽂으면 libcamera 카메라 인덱스가 밀린다.** libcamera는 UVC 웹캠도
  함께 열거해서, 웹캠이 꽂힌 상태에서는 인덱스 0이 웹캠, 1이 imx219가 된다.
  `camera_ros` 는 인덱스 0을 기본으로 고르므로 `camera` 파라미터를 비워두면
  CSI 카메라 대신 웹캠이 열리고, 웹캠은 RGB888을 지원하지 않아
  `unsupported pixel format "RGB888"` 로 노드가 죽는다. `sensors.launch.py` 는
  device-tree 경로(`/base/soc/i2c0mux/i2c@1/imx219@10`)로 CSI 카메라를 고정한다.

## 트러블슈팅

**IMU가 안 잡힘**
```bash
i2cdetect -y 1        # 0x68 이 보여야 정상
```

**카메라가 안 잡힘**
```bash
ros2 run camera_ros camera_node   # 로그에 imx219 가 나오는지 확인
ros2 topic hz /camera/image_raw   # 820x616 RGB888
```
`cam -l` (libcamera CLI)은 이 환경에서 세그폴트한다. ROS 노드 경로는 정상이니 무시해도 된다.

**모터가 안 움직임**
`pid_controller_node` 와 `motor_driver_node` 가 동시에 떠 있는지 먼저 확인한다.
GPIO 충돌이 가장 흔한 원인이다.

```bash
ps aux | grep -E "pid_controller_node|motor_driver_node"
```

**노드를 껐는데 토픽이 계속 나옴**
`ros2 run` 을 백그라운드로 띄웠다 죽이면 자식 프로세스가 남는 경우가 있다.

```bash
pkill -f mpu6050_node
pkill -f camera_node
```
