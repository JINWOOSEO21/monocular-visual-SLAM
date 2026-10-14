#!/usr/bin/env python3
"""mono_node 가 종료 시 저장한 키프레임 궤적과 맵 포인트를 그린다.

입력 (둘 다 같은 맵, orb_world 프레임, 스케일 없는 ORB 단위)
    --trajectory  키프레임 궤적, TUM 형식: t tx ty tz qx qy qz qw  (T_world_cam)
    --points      맵 포인트, ASCII PLY: x y z [observations] [t_first]
    --events      (선택) 그 실행의 launch.log. mono_node 가 남긴 새 맵 / 맵 병합 / 루프 클로저
                  시각을 읽어 그림에 표시한다. launch.log 의 줄 앞 시각(벽시계)을 쓰므로
                  실시간 실행 기준이다 (rosbag 재생이면 시각이 맞지 않는다).

출력 (여러 개를 함께 줄 수 있다. 하나도 없으면 세 가지를 모두 --outdir 에
<이름>_map.png / <이름>_run.gif / <이름>_viewer.html 로 저장하고, --show 면 정적 그림을 창으로 띄운다)
    --out      정적 그림 (.png / .pdf). 위에서 본 그림 + 옆에서 본 그림
    --animate  주행 애니메이션 (.gif). 궤적이 시간 순서대로 그려진다
    --html     브라우저에서 돌려 보는 3D 뷰어 (자체 완결 HTML, Plotly 는 CDN 에서 읽는다)

orb_world 는 맵을 시작한 키프레임의 카메라 광학 프레임(x 오른쪽, y 아래, z 앞)이다.
카메라가 기울어 달려 있으면 이 프레임도 기울어 있어서, 평지를 달려도 궤적이 위아래로
움직인 것처럼 보인다. 그래서 그리기 전에 바닥을 수평으로 돌린다 (--level):

    pca    궤적에 평면을 맞춰 그 법선을 위로 삼는다 (기본). 평지 주행이면 장착 각도를
           몰라도 된다. 궤적이 거의 직선이면 평면이 정해지지 않아 mount 로 넘어간다.
    mount  첫 키프레임의 카메라가 --pitch-deg 만큼 위로 들려 있고 roll 이 0 이라고 본다.
           sensor_tf.launch.py 의 base_link -> camera_link pitch 와 같은 값을 준다.
    none   카메라가 수평이라고 본다 (= mount, pitch 0).

어느 방식이든 pca 로 추정한 카메라 들림 각도를 함께 출력하므로, TF 의 pitch 를
검증하는 데 쓸 수 있다.

수평 정렬 후 좌표는 첫 키프레임의 로봇 기준 x 전방, y 왼쪽, z 위이고, 스케일
("1 ORB 단위 = s m")을 곱해 미터로 그린다. 스케일은 --scale 로 주거나, 주지 않으면
scale_aligner 가 종료 시 궤적 파일 옆에 저장한 scale.yaml(--scale-file)에서 현재 맵의
값을 읽는다. 둘 다 없으면 ORB 단위 그대로 그린다.

--min-height / --max-height 를 주면 그 높이 범위(양 끝 포함)의 맵 포인트만 모든 출력에 그린다.
높이는 정렬 후 z 라서 0 이 출발 시점의 카메라 높이다 (바닥이 아니다). 낮은 점들은 가구·벽의
아랫부분이라 로봇이 다닐 수 있는 공간의 경계를 보여 준다.

애니메이션과 3D 뷰어는 맵 포인트를 처음 관측된 시각에 하나씩 나타나게 해서 지도가 쌓여
가는 모습을 보여 준다 (--static-points 면 처음부터 다 보인다). 그 시각은
    t_first 가 PLY 에 있으면   mono_node 가 기록한 값 (그 점을 관측한 가장 이른 키프레임 시각)
    없으면 (예전 PLY)         추정값: 점이 키프레임 카메라 시야 안(--settings 의 내부 파라미터)에
                              처음 들어온 시각. 가림은 따지지 않고, 그 점이 보인 가장 가까운
                              거리의 --reveal-range 배보다 멀면 안 보인 것으로 친다.

    ros2 run orb_slam plot_map --show          # 스케일은 scale.yaml 에서
    ros2 run orb_slam plot_map --scale 2.39 --show
    ros2 run orb_slam plot_map --level mount --pitch-deg 20 --out map.png
    ros2 run orb_slam plot_map --scale 2.6 --events ~/.ros/log/<실행>/launch.log --name test_1820
    ros2 run orb_slam plot_map --min-height 0 --max-height 0.05 --out low_map.png --animate low_run.gif
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

ORB_DIR = Path("~/.ros/orbslam3").expanduser()

# launch.log 에서 찾을 mono_node 메시지 -> 이벤트 종류
EVENT_PATTERNS = (
    ("new_map", "새 맵 초기화"),
    ("map_switch", "현재 맵이 바뀜"),
    ("loop", "*Loop detected"),
)
EVENT_LABELS_EN = {"new_map": "new map", "map_switch": "map merge", "loop": "loop closure"}


# ── 입력 ──────────────────────────────────────────────────────────
def load_trajectory(path):
    """(시각, 위치 Nx3, 쿼터니언 Nx4 [qx qy qz qw]) 를 돌려준다."""
    data = np.loadtxt(path, ndmin=2)
    if data.size == 0:
        raise SystemExit(
            f"{path} 가 비어 있다. 종료 시점의 현재 맵에 키프레임이 없었다 (막 새로 만든 맵)."
        )
    return data[:, 0], data[:, 1:4], data[:, 4:8]


def load_ply(path):
    """ASCII PLY 의 vertex 를 읽어 {property 이름: 배열} 로 돌려준다."""
    with Path(path).open() as f:
        if f.readline().strip() != "ply":
            raise SystemExit(f"{path} 는 PLY 파일이 아니다")
        names, count = [], 0
        for line in f:
            tok = line.split()
            if not tok:
                continue
            if tok[0] == "format" and tok[1] != "ascii":
                raise SystemExit(f"{path}: ASCII PLY 만 읽는다 (format {tok[1]})")
            if tok[0] == "element" and tok[1] == "vertex":
                count = int(tok[2])
            elif tok[0] == "property":
                names.append(tok[-1])
            elif tok[0] == "end_header":
                break
        rows = np.loadtxt(f, ndmin=2, max_rows=count) if count else np.empty((0, len(names)))
    return {name: rows[:, i] for i, name in enumerate(names)}


def load_scale(path, trajectory):
    """scale_aligner 가 저장한 scale.yaml 에서 현재 맵(=궤적 파일의 맵)의 스케일을 읽는다."""
    import yaml

    path = Path(path).expanduser()
    if not path.is_file():
        print(f"스케일 파일이 없어 ORB 단위로 그린다: {path} (--scale 로 줄 수 있다)")
        return None
    data = yaml.safe_load(path.read_text()) or {}
    key = data.get("current_map_key")
    entry = (data.get("maps") or {}).get(key)
    if not entry:
        print(f"{path}: 현재 맵(key {key})은 스케일이 정해지기 전에 끝났다 -> ORB 단위로 그린다")
        return None
    gap = abs(path.stat().st_mtime - Path(trajectory).stat().st_mtime)
    if gap > 120:
        print(f"주의: {path.name} 와 궤적 파일의 저장 시각이 {gap / 60:.0f} 분 다르다. "
              "다른 실행의 스케일일 수 있다")
    print(f"스케일 {entry['scale']:.3f} m / ORB 단위 ({path.name}, 맵 key {key}, "
          f"{entry.get('segments_used')}/{entry.get('segments')} 구간)")
    return float(entry["scale"])


def load_events(path):
    """launch.log 에서 (시각, 종류) 목록을 읽는다. 줄 앞의 숫자가 기록 시각이다."""
    events = []
    with Path(path).expanduser().open(errors="replace") as f:
        for line in f:
            head, _, rest = line.partition(" ")
            try:
                t = float(head)
            except ValueError:
                continue
            for kind, key in EVENT_PATTERNS:
                if key in rest:
                    events.append((t, kind))
                    break
    return events


# ── 수평 정렬 ─────────────────────────────────────────────────────
def fit_up(positions):
    """궤적 평면의 법선(위쪽)과, 평면이 잘 정해졌는지(두 번째/첫 번째 특이값 비)를 구한다."""
    centered = positions - positions.mean(axis=0)
    _, s, vt = np.linalg.svd(centered, full_matrices=False)
    up = vt[2]
    if up[1] > 0:  # 광학 프레임에서 위는 -y 쪽이다
        up = -up
    spread = s[1] / s[0] if s[0] > 0 else 0.0
    return up, spread


def mount_axes(pitch_deg):
    """카메라가 pitch_deg 만큼 위로 들린 장착에서 로봇 축(전방, 왼쪽, 위)을 광학 프레임으로."""
    th = math.radians(pitch_deg)
    forward = np.array([0.0, math.sin(th), math.cos(th)])
    left = np.array([-1.0, 0.0, 0.0])
    up = np.array([0.0, -math.cos(th), math.sin(th)])
    return np.vstack([forward, left, up])


def axes_from_up(up):
    """위쪽 벡터로 수평 프레임을 만든다. 전방은 첫 카메라 광축(z)을 바닥에 투영한 방향."""
    forward = np.array([0.0, 0.0, 1.0]) - up[2] * up
    forward /= np.linalg.norm(forward)
    left = np.cross(up, forward)
    return np.vstack([forward, left, up])


def elevation_deg(up):
    """광학 프레임의 위쪽 벡터로부터 카메라 광축이 수평에서 들린 각도."""
    return math.degrees(math.atan2(up[2], -up[1]))


def rotations(quats):
    """쿼터니언 [qx qy qz qw] 들을 회전 행렬 Nx3x3 로."""
    x, y, z, w = quats.T
    return np.stack([
        np.column_stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)]),
        np.column_stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)]),
        np.column_stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]),
    ], axis=1)


def optical_axis(quats):
    """T_world_cam 쿼터니언들에서 카메라 광축(카메라 z)의 월드 방향."""
    return rotations(quats)[:, :, 2]


# ── 맵 포인트가 처음 보인 시각 ─────────────────────────────────────
def default_settings():
    """orb_slam 패키지의 카메라 설정 yaml (설치본, 없으면 소스 트리)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("orb_slam")) / "config" / "imx219_820x616.yaml"
    except Exception:
        return Path(__file__).resolve().parent.parent / "config" / "imx219_820x616.yaml"


def load_intrinsics(path):
    """ORB-SLAM3 설정 yaml 에서 (fx, fy, cx, cy, width, height). OpenCV yaml 이라 줄 단위로 읽는다."""
    want = {"Camera1.fx": "fx", "Camera1.fy": "fy", "Camera1.cx": "cx", "Camera1.cy": "cy",
            "Camera.width": "w", "Camera.height": "h"}
    got = {}
    for line in Path(path).expanduser().read_text().splitlines():
        key, _, val = line.partition(":")
        if key.strip() in want:
            got[want[key.strip()]] = float(val.split("#")[0])
    missing = set(want.values()) - set(got)
    if missing:
        raise SystemExit(f"{path} 에 카메라 파라미터가 없다: {sorted(missing)}")
    return got


def estimate_first_seen(pts, stamps, traj, quats, intr, reveal_range):
    """각 점이 키프레임 카메라 시야에 처음 들어온 시각 (stamps 와 같은 시간축).

    시야 = 카메라 앞, 이미지 안, 거리 <= reveal_range x (그 점이 보인 가장 가까운 거리).
    ORB 특징은 스케일 피라미드 범위 안에서만 다시 잡히므로 멀리서 시야에 걸쳤을 뿐인 점을
    너무 일찍 나타나게 하지 않으려는 것이다. 어느 키프레임에서도 시야에 없으면 가장 가까운
    키프레임의 시각을 쓴다.
    """
    rots = rotations(quats)
    n = len(pts)
    first = np.full(n, -1)
    dist = np.empty((len(traj), n))
    vis = np.zeros((len(traj), n), dtype=bool)
    for k, (t, r) in enumerate(zip(traj, rots)):
        pc = (pts - t) @ r  # 카메라 프레임: R^T (p - t)
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = intr["fx"] * pc[:, 0] / z + intr["cx"]
            v = intr["fy"] * pc[:, 1] / z + intr["cy"]
        vis[k] = (z > 1e-6) & (u >= 0) & (u < intr["w"]) & (v >= 0) & (v < intr["h"])
        dist[k] = np.linalg.norm(pc, axis=1)
    nearest_seen = np.where(vis, dist, np.inf).min(axis=0)
    ok = vis & (dist <= reveal_range * nearest_seen)
    seen = ok.any(axis=0)
    first[seen] = ok[:, seen].argmax(axis=0)
    first[~seen] = dist[:, ~seen].argmin(axis=0)
    return stamps[first], int((~seen).sum())


# ── 실행 데이터 정리 ──────────────────────────────────────────────
class Run:
    """수평 정렬·스케일을 적용한 한 실행의 궤적, 진행 방향, 맵 포인트, 이벤트."""

    def __init__(self, args):
        self.name = args.name or Path(args.trajectory).stem
        self.scale = args.scale
        self.scale_approx = args.scale_approx
        self.unit = "m" if args.scale != 1.0 else "ORB units"
        stamps, traj, quats = load_trajectory(args.trajectory)

        self.pca_up, self.spread = fit_up(traj) if len(traj) >= 3 else (None, 0.0)
        level = args.level
        if level == "pca" and (self.pca_up is None or self.spread < 0.05):
            print("궤적이 거의 직선이라 바닥 평면을 정할 수 없다 -> --level mount 로 그린다")
            level = "mount"
        if level == "pca":
            axes = axes_from_up(self.pca_up)
        elif level == "mount":
            axes = mount_axes(args.pitch_deg)
        else:
            axes = mount_axes(0.0)
        self.level = level
        self.pitch_deg = args.pitch_deg

        origin = traj[0]
        self.t = stamps - stamps[0]
        self.t0 = stamps[0]
        self.xyz = (traj - origin) @ axes.T * args.scale
        heading = optical_axis(quats) @ axes.T
        self.yaw = np.arctan2(heading[:, 1], heading[:, 0])
        steps = np.linalg.norm(np.diff(self.xyz, axis=0), axis=1)
        self.path_len = float(steps.sum())

        self.pts = np.empty((0, 3))
        self.obs = np.empty(0)
        self.t_seen = np.empty(0)  # 점이 처음 관측된 시각 (첫 키프레임 기준 초)
        self.seen_source = None    # "recorded" | "estimated"
        self.n_points_file = 0
        if Path(args.points).is_file():
            ply = load_ply(args.points)
            pts = np.column_stack([ply["x"], ply["y"], ply["z"]]) if ply else np.empty((0, 3))
            obs = ply.get("observations", np.full(len(pts), args.min_obs))
            keep = obs >= args.min_obs
            self.n_points_file = len(pts)
            self.pts = (pts[keep] - origin) @ axes.T * args.scale
            self.obs = obs[keep]
            print(f"맵 포인트 {len(pts)} 개 중 관측 {args.min_obs} 회 이상 {len(self.pts)} 개")
            if "t_first" in ply:
                t_first = ply["t_first"][keep]
                self.t_seen = np.where(np.isfinite(t_first), t_first - stamps[0], self.t[-1])
                self.seen_source = "recorded"
                print("맵 포인트가 나타나는 시각: PLY 의 t_first (mono_node 기록)")
            elif len(self.pts):
                intr = load_intrinsics(args.settings or default_settings())
                t_first, unseen = estimate_first_seen(
                    pts[keep], stamps, traj, quats, intr, args.reveal_range)
                self.t_seen = t_first - stamps[0]
                self.seen_source = "estimated"
                print("맵 포인트가 나타나는 시각: 키프레임 시야로 추정 (PLY 에 t_first 없음)"
                      + (f", 시야에 한 번도 안 든 점 {unseen} 개는 가장 가까운 키프레임 시각"
                         if unseen else ""))
            self.t_seen = np.clip(self.t_seen, 0.0, self.t[-1])
            zmin = getattr(args, "min_height", None)
            zmax = getattr(args, "max_height", None)
            if (zmin is not None or zmax is not None) and len(self.pts):
                z = self.pts[:, 2]
                band = ((z >= zmin) if zmin is not None else True) & \
                       ((z <= zmax) if zmax is not None else True)
                print(f"높이 {'' if zmin is None else f'{zmin:g} 이상 '}"
                      f"{'' if zmax is None else f'{zmax:g} 이하 '}({self.unit}) "
                      f"맵 포인트 {int(band.sum())} 개만 그린다")
                self.pts, self.obs, self.t_seen = self.pts[band], self.obs[band], self.t_seen[band]
        else:
            print(f"맵 포인트 파일이 없어 궤적만 그린다: {args.points}")

        # 이벤트는 궤적 시간 안의 것만, 가장 가까운 키프레임에 붙인다. 첫 "새 맵" 은 이 지도의
        # 초기화 자체라 시작 표시와 같으므로 뺀다 (첫 키프레임 시각은 초기화를 기다리기 시작한
        # 프레임이라 초기화 메시지보다 수십 초 이를 수 있다).
        self.events = []
        if args.events:
            first_map_skipped = False
            for ts, kind in load_events(args.events):
                if kind == "new_map" and not first_map_skipped:
                    first_map_skipped = True
                    continue
                rel = ts - self.t0
                if rel < -1.0 or rel > self.t[-1] + 1.0:
                    continue
                k = int(np.clip(np.searchsorted(self.t, rel), 0, len(self.t) - 1))
                self.events.append((rel, kind, k))
            print(f"이벤트 {len(self.events)} 개: "
                  + ", ".join(f"{EVENT_LABELS_EN[k]} {t:.1f}s" for t, k, _ in self.events))

    def summary(self):
        print(f"키프레임 {len(self.xyz)} 개, {self.t[-1]:.1f} s, "
              f"경로 길이 {self.path_len:.2f} {self.unit}")
        print(f"수평 정렬: {self.level}"
              + (f" (pitch {self.pitch_deg:.1f} deg)" if self.level == "mount" else ""))
        if self.pca_up is not None:
            print(f"궤적 평면으로 추정한 카메라 들림 각도 {elevation_deg(self.pca_up):.1f} deg, "
                  f"좌우 기울기 {math.degrees(math.asin(self.pca_up[0])):.1f} deg "
                  f"(평면 신뢰도 {self.spread:.2f}, 0.05 미만이면 불확실)")
        print(f"정렬 후 키프레임 높이: 표준편차 {self.xyz[:, 2].std():.3f} {self.unit}, "
              f"범위 {np.ptp(self.xyz[:, 2]):.3f} {self.unit}  (평지 주행이면 0 에 가까워야 한다)")

    def view_box(self, margin=None):
        """궤적을 중심으로 한 위에서 본 그림의 범위. 멀리 튄 맵 포인트에 끌려가지 않는다."""
        lo, hi = self.xyz[:, :2].min(axis=0), self.xyz[:, :2].max(axis=0)
        if margin is None:
            margin = 0.35 * float(max(hi - lo))
        return lo - margin, hi + margin

    def in_box(self, lo, hi):
        """맵 포인트 중 위에서 본 범위 안에 있는 것의 마스크."""
        return np.all((self.pts[:, :2] >= lo) & (self.pts[:, :2] <= hi), axis=1)

    def points_in(self, lo, hi):
        return self.pts[self.in_box(lo, hi)]

    def stats_line(self):
        scale = f"{'≈' if self.scale_approx else ''}{self.scale:g} m per ORB unit"
        return (f"{self.t[-1]:.0f} s  ·  {self.path_len:.1f} {self.unit} driven  ·  "
                f"{len(self.xyz)} keyframes  ·  {len(self.pts)} map points  ·  scale {scale}")


# ── 정적 그림 ─────────────────────────────────────────────────────
def _height_limits(z):
    if len(z) == 0:
        return 0.0, 1.0
    lo, hi = np.percentile(z, [2, 98])
    return float(lo), float(max(hi, lo + 1e-6))


def plot_static(run, out, zrange=None):
    import matplotlib
    if out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lo, hi = run.view_box()
    fig, (top, side) = plt.subplots(
        1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [3, 2]}, constrained_layout=True
    )
    fig.suptitle(run.name, fontsize=15, fontweight="bold")

    pts = run.points_in(lo, hi)
    zlo, zhi = _height_limits(pts[:, 2])
    top_pts = pts
    if zrange:
        top_pts = pts[(pts[:, 2] >= zrange[0]) & (pts[:, 2] <= zrange[1])]
    if len(top_pts):
        order = np.argsort(top_pts[:, 2])  # 높은 점이 위에 오게
        sc = top.scatter(top_pts[order, 0], top_pts[order, 1], c=top_pts[order, 2], s=1.5,
                         cmap="viridis", vmin=zlo, vmax=zhi, alpha=0.75, linewidths=0)
        fig.colorbar(sc, ax=top, shrink=0.75, pad=0.01, label=f"map point height [{run.unit}]")
        side.scatter(pts[:, 0], pts[:, 2], c=pts[:, 2], s=1.5, cmap="viridis",
                     vmin=zlo, vmax=zhi, alpha=0.6, linewidths=0)

    for ax, j in ((top, 1), (side, 2)):
        ax.plot(run.xyz[:, 0], run.xyz[:, j], "-", c="#d1495b", lw=1.8, zorder=3,
                label="keyframe trajectory")
        ax.plot(run.xyz[0, 0], run.xyz[0, j], "o", ms=10, mfc="white", mec="#222", mew=2,
                zorder=5, label="start")
        ax.plot(run.xyz[-1, 0], run.xyz[-1, j], "s", ms=9, c="#222", zorder=5, label="end")
        ax.grid(True, lw=0.3, alpha=0.6)
        ax.set_xlabel(f"x forward [{run.unit}]")

    # 진행 방향 화살표 (키프레임 몇 개마다)
    step = max(1, len(run.xyz) // 28)
    idx = np.arange(0, len(run.xyz), step)
    arrow = 0.04 * float(max(hi - lo))
    top.quiver(run.xyz[idx, 0], run.xyz[idx, 1], np.cos(run.yaw[idx]) * arrow,
               np.sin(run.yaw[idx]) * arrow, angles="xy", scale_units="xy", scale=1,
               color="#d1495b", width=0.003, zorder=4)

    markers = {"map_switch": ("D", "#e09f3e", 9), "loop": ("*", "#6a4c93", 18),
               "new_map": ("X", "#555555", 9)}
    seen = set()
    for t, kind, k in run.events:
        m, color, size = markers[kind]
        label = EVENT_LABELS_EN[kind] if kind not in seen else None
        seen.add(kind)
        for ax, j in ((top, 1), (side, 2)):
            ax.plot(run.xyz[k, 0], run.xyz[k, j], m, c=color, ms=size, mec="#222", mew=0.8,
                    zorder=6, label=label if ax is top else None)
        if kind == "new_map":  # 보통 곧바로 병합이 뒤따르므로 라벨은 병합 쪽에만 단다
            continue
        top.annotate(f"{EVENT_LABELS_EN[kind]}\n{t:.0f} s", run.xyz[k, :2], xytext=(8, 8),
                     textcoords="offset points", fontsize=8,
                     bbox={"boxstyle": "round,pad=0.25", "fc": "white", "ec": color, "alpha": 0.9})

    top.set_xlim(lo[0], hi[0])
    top.set_ylim(lo[1], hi[1])
    top.set_aspect("equal", adjustable="datalim")
    top.set_ylabel(f"y left [{run.unit}]")
    top.set_title("Top view")
    top.legend(loc="upper right", fontsize=8, framealpha=0.9)
    top.text(0.01, 0.01, run.stats_line(), transform=top.transAxes, fontsize=8, va="bottom",
             bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#bbb", "alpha": 0.9})
    side.set_xlim(lo[0], hi[0])
    side.set_ylabel(f"z up [{run.unit}]")
    side.set_title("Side view")

    if out:
        fig.savefig(out, dpi=200)
        print(f"저장: {out}")
    else:
        plt.show()
    plt.close(fig)


# ── 애니메이션 ────────────────────────────────────────────────────
FRESH_COLOR = "#f28e2b"  # 방금 관측된 맵 포인트


def animate(run, out, speed, fps, reveal=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    lo, hi = run.view_box()
    fig, ax = plt.subplots(figsize=(8, 7), constrained_layout=True)
    box = run.in_box(lo, hi)
    pts = run.pts[box]
    reveal = reveal and run.seen_source is not None and len(pts) > 0
    if reveal:
        # 처음 관측된 순서로 정렬해 두면 시각 tf 까지 보인 점은 앞쪽 n 개다
        order = np.argsort(run.t_seen[box], kind="stable")
        pts, t_seen = pts[order], run.t_seen[box][order]
        zlo, zhi = _height_limits(pts[:, 2])
        seen_sc = ax.scatter(np.empty(0), np.empty(0), c=np.empty(0), s=1.4, cmap="viridis",
                             vmin=zlo, vmax=zhi, alpha=0.55, linewidths=0)
        fresh_sc = ax.scatter(np.empty(0), np.empty(0), s=4, c=FRESH_COLOR, alpha=0.9,
                              linewidths=0, zorder=2)
        fresh_window = 0.75 * speed  # 재생 시간으로 0.75 초 동안 '방금 관측' 색으로 둔다
    elif len(pts):
        zlo, zhi = _height_limits(pts[:, 2])
        order = np.argsort(pts[:, 2])
        ax.scatter(pts[order, 0], pts[order, 1], c=pts[order, 2], s=1.2, cmap="viridis",
                   vmin=zlo, vmax=zhi, alpha=0.35, linewidths=0)
    ax.plot(run.xyz[0, 0], run.xyz[0, 1], "o", ms=9, mfc="white", mec="#222", mew=2, zorder=5)
    (line,) = ax.plot([], [], "-", c="#d1495b", lw=2.0, zorder=3)
    (head,) = ax.plot([], [], "o", c="#d1495b", ms=8, mec="#222", zorder=6)
    arrow_len = 0.06 * float(max(hi - lo))
    quiv = ax.quiver([0], [0], [0], [0], angles="xy", scale_units="xy", scale=1,
                     color="#222", width=0.006, zorder=7)
    clock = ax.text(0.02, 0.97, "", transform=ax.transAxes, fontsize=11, va="top",
                    family="monospace", bbox={"fc": "white", "ec": "#bbb", "alpha": 0.9})
    banner = ax.text(0.5, 0.06, "", transform=ax.transAxes, fontsize=13, ha="center",
                     fontweight="bold", color="white",
                     bbox={"boxstyle": "round,pad=0.4", "fc": "#6a4c93", "ec": "none"})
    colors = {"map_switch": "#e09f3e", "loop": "#6a4c93", "new_map": "#555555"}
    marks = {"map_switch": "D", "loop": "*", "new_map": "X"}
    event_artists = []
    for _t, kind, k in run.events:
        (a,) = ax.plot(run.xyz[k, 0], run.xyz[k, 1], marks[kind], c=colors[kind],
                       ms=16 if kind == "loop" else 9, mec="#222", zorder=8, visible=False)
        event_artists.append(a)

    ax.set_xlim(lo[0], hi[0])
    ax.set_ylim(lo[1], hi[1])
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, lw=0.3, alpha=0.6)
    ax.set_xlabel(f"x forward [{run.unit}]")
    ax.set_ylabel(f"y left [{run.unit}]")
    ax.set_title(f"{run.name} — ORB-SLAM3 {'map building' if reveal else 'keyframe trajectory'}"
                 f" ({speed:g}x)")
    if reveal:
        ax.plot([], [], "o", c=FRESH_COLOR, ms=4, label="just observed")
        ax.plot([], [], "o", c=plt.get_cmap("viridis")(0.6), ms=4, label="map points (by height)")
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

    duration = float(run.t[-1])
    n_frames = int(duration / speed * fps) + 1
    hold = int(fps * 2)  # 끝에서 2 초 멈춘다

    def draw(i):
        tf = min(i, n_frames - 1) * speed / fps
        k = int(np.searchsorted(run.t, tf, side="right") - 1)
        k = max(k, 0)
        line.set_data(run.xyz[: k + 1, 0], run.xyz[: k + 1, 1])
        head.set_data([run.xyz[k, 0]], [run.xyz[k, 1]])
        quiv.set_offsets([[run.xyz[k, 0], run.xyz[k, 1]]])
        quiv.set_UVC([np.cos(run.yaw[k]) * arrow_len], [np.sin(run.yaw[k]) * arrow_len])
        dist = np.linalg.norm(np.diff(run.xyz[: k + 1], axis=0), axis=1).sum()
        clock_text = f"t = {tf:6.1f} s   {dist:5.1f} {run.unit}"
        if reveal:
            n = int(np.searchsorted(t_seen, tf, side="right"))
            m = int(np.searchsorted(t_seen, tf - fresh_window, side="right"))
            seen_sc.set_offsets(pts[:m, :2])
            seen_sc.set_array(pts[:m, 2])
            fresh_sc.set_offsets(pts[m:n, :2] if n > m else np.empty((0, 2)))
            clock_text += f"\n{n:5d} map points"
        clock.set_text(clock_text)
        text, color = "", None
        for (t, kind, _k), artist in zip(run.events, event_artists):
            artist.set_visible(t <= tf)
            if 0 <= tf - t < 1.5 * speed:  # 이벤트 뒤 재생 시간으로 1.5 초 동안 배너를 띄운다
                text, color = EVENT_LABELS_EN[kind], colors[kind]
        banner.set_text(text)
        banner.set_visible(bool(text))
        if color:
            banner.get_bbox_patch().set_facecolor(color)
        return line, head, quiv, clock, banner, *event_artists

    anim = FuncAnimation(fig, draw, frames=n_frames + hold, blit=False)
    anim.save(out, writer=PillowWriter(fps=fps), dpi=90)
    plt.close(fig)
    print(f"저장: {out} ({n_frames + hold} 프레임, {(n_frames + hold) / fps:.1f} s)")


# ── 3D 뷰어 HTML ──────────────────────────────────────────────────
def export_html(run, out):
    lo, hi = run.view_box(margin=None)
    # 3D 에서는 궤적 주변을 조금 더 넓게 남기되, 멀리 튄 점은 뺀다
    span = float(max(hi - lo))
    keep = run.in_box(lo - 0.3 * span, hi + 0.3 * span)
    if keep.any():
        zlo, zhi = np.percentile(run.pts[keep, 2], [0.5, 99.5])
        keep &= (run.pts[:, 2] >= zlo) & (run.pts[:, 2] <= zhi)
    pts = run.pts[keep]
    if run.seen_source is not None:
        t_seen = run.t_seen[keep]
        order = np.argsort(t_seen, kind="stable")  # 뷰어가 앞쪽 n 개만 잘라 쓴다
        pts, t_seen = pts[order], t_seen[order]
    else:
        t_seen = np.zeros(len(pts))
    data = {
        "name": run.name,
        "unit": run.unit,
        "scale": run.scale,
        "scaleApprox": run.scale_approx,
        "duration": round(float(run.t[-1]), 2),
        "pathLength": round(run.path_len, 2),
        "elevation": round(elevation_deg(run.pca_up), 1) if run.pca_up is not None else None,
        "nPointsFile": run.n_points_file,
        "traj": [[round(float(t), 2), *map(lambda v: round(float(v), 3), p), round(float(y), 3)]
                 for t, p, y in zip(run.t, run.xyz, run.yaw)],
        "points": [[*(round(float(v), 3) for v in p), round(float(t), 2)]
                   for p, t in zip(pts, t_seen)],
        "seenSource": run.seen_source,
        "events": [{"t": round(t, 1), "kind": kind, "k": k} for t, kind, k in run.events],
    }
    blob = json.dumps(data, ensure_ascii=False, separators=(",", ":")).replace("<", "\\u003c")
    html = HTML_TEMPLATE.replace("__TITLE__", f"{run.name} 주행 지도").replace("__DATA__", blob)
    Path(out).write_text(html, encoding="utf-8")
    print(f"저장: {out} ({len(html) / 1024:.0f} KB, 맵 포인트 {len(pts)} 개)")


HTML_TEMPLATE = r"""<title>__TITLE__</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+KR:wght@400;600&display=swap">
<style>
/* layout: one-column instrument sheet - header with spec strip, 3D plot, timeline scrubber with event ticks, notes */
:root {
  --bg: #eef2f4; --panel: #ffffff; --ink: #14202a; --muted: #566573; --line: #d3dbe1;
  --accent: #c2364a; --merge: #b7791f; --loop: #5b3f8c; --pt-low: #b9c7d1; --pt-high: #1f5d75;
  --fresh: #e8731a;
  --font-body: "IBM Plex Sans KR", "Noto Sans KR", "Apple SD Gothic Neo", system-ui, sans-serif;
  --font-mono: "IBM Plex Mono", ui-monospace, "SFMono-Regular", Menlo, monospace;
}
@media (prefers-color-scheme: dark) {
  :root:not([data-theme="light"]) {
    --bg: #0d1418; --panel: #141d23; --ink: #e3e9ed; --muted: #95a3ae; --line: #25323b;
    --accent: #ff6b7d; --merge: #f0b549; --loop: #b394f0; --pt-low: #34495a; --pt-high: #8fdcf2;
  --fresh: #ffa24c;
    color-scheme: dark;
  }
}
:root[data-theme="dark"] {
  --bg: #0d1418; --panel: #141d23; --ink: #e3e9ed; --muted: #95a3ae; --line: #25323b;
  --accent: #ff6b7d; --merge: #f0b549; --loop: #b394f0; --pt-low: #34495a; --pt-high: #8fdcf2;
  --fresh: #ffa24c;
  color-scheme: dark;
}
* { box-sizing: border-box; }
body { background: var(--bg); color: var(--ink); font-family: var(--font-body); font-size: 15px; line-height: 1.55; }
.page { max-width: 1120px; margin: 0 auto; padding-inline: 16px; padding-block: 28px 40px; display: grid; gap: 20px; }
header { display: grid; gap: 6px; }
.eyebrow { font-size: 13px; letter-spacing: 0.02em; color: var(--muted); }
h1 { margin: 0; font-size: clamp(26px, 4vw, 38px); font-weight: 600; text-wrap: balance; letter-spacing: -0.01em; }
.lede { margin: 0; color: var(--muted); max-width: 65ch; }
.specs { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 1px; background: var(--line); border: 1px solid var(--line); border-radius: 10px; overflow: hidden; margin: 0; }
.specs div { background: var(--panel); padding: 12px 14px; display: grid; gap: 2px; min-width: 0; }
.specs dt { font-size: 12px; color: var(--muted); }
.specs dd { margin: 0; font-family: var(--font-mono); font-size: 20px; font-weight: 500; font-variant-numeric: tabular-nums; }
.specs dd small { font-size: 12px; color: var(--muted); font-weight: 400; }
.viewer { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; overflow: hidden; }
.viewer-bar { display: flex; flex-wrap: wrap; gap: 8px 18px; align-items: center; padding: 10px 14px; border-bottom: 1px solid var(--line); font-size: 13px; }
.viewer-bar label { display: inline-flex; gap: 6px; align-items: center; cursor: pointer; }
.viewer-bar .hint { margin-left: auto; color: var(--muted); }
.swatch { width: 12px; height: 12px; border-radius: 3px; display: inline-block; }
#plot { width: 100%; height: clamp(380px, 68vh, 720px); }
#plot .fallback { padding: 40px 20px; color: var(--muted); }
.timeline { background: var(--panel); border: 1px solid var(--line); border-radius: 10px; padding: 14px; display: grid; gap: 12px; }
.scrub { display: grid; grid-template-columns: auto 1fr auto; gap: 12px; align-items: center; }
.scrub button { font: inherit; font-size: 13px; padding: 7px 14px; border-radius: 999px; border: 1px solid var(--ink); background: var(--ink); color: var(--panel); cursor: pointer; min-width: 74px; }
.scrub button:focus-visible, .events button:focus-visible, input:focus-visible { outline: 2px solid var(--accent); outline-offset: 2px; }
.track { position: relative; padding-top: 16px; min-width: 0; }
.ticks { position: absolute; left: 0; right: 0; top: 0; height: 14px; }
.ticks span { position: absolute; top: 0; width: 3px; height: 12px; border-radius: 2px; transform: translateX(-50%); }
input[type=range] { width: 100%; accent-color: var(--accent); margin: 0; }
.clock { font-family: var(--font-mono); font-size: 13px; font-variant-numeric: tabular-nums; color: var(--muted); white-space: nowrap; }
.events { display: flex; flex-wrap: wrap; gap: 8px; }
.events button { font: inherit; font-size: 13px; display: inline-flex; gap: 8px; align-items: center; padding: 5px 12px 5px 8px; border-radius: 999px; border: 1px solid var(--line); background: transparent; color: var(--ink); cursor: pointer; }
.events button .t { font-family: var(--font-mono); color: var(--muted); font-variant-numeric: tabular-nums; }
.events button .dot { width: 10px; height: 10px; border-radius: 50%; }
.notes { color: var(--muted); font-size: 13px; display: grid; gap: 6px; max-width: 80ch; }
.notes p { margin: 0; }
.notes code { font-family: var(--font-mono); font-size: 12px; background: var(--panel); border: 1px solid var(--line); border-radius: 4px; padding: 1px 5px; overflow-wrap: anywhere; }
@media (max-width: 560px) { .scrub { grid-template-columns: auto 1fr; } .clock { grid-column: 1 / -1; } .viewer-bar .hint { margin-left: 0; } }
</style>

<div class="page">
  <header>
    <div class="eyebrow">ORB-SLAM3 · 단안 카메라 · 키프레임 궤적과 맵 포인트</div>
    <h1 id="title">주행 지도</h1>
    <p class="lede" id="lede">라즈베리파이 RC카가 단안 카메라만으로 만든 지도입니다. 끌어서 돌리고, 휠로 확대하고, 아래 타임라인으로 주행을 다시 볼 수 있습니다.</p>
  </header>

  <dl class="specs" id="specs"></dl>

  <section class="viewer" aria-label="3D 지도">
    <div class="viewer-bar">
      <label><input type="checkbox" id="show-points" checked> <span class="swatch" id="sw-points"></span> 맵 포인트</label>
      <label><input type="checkbox" id="show-traj" checked> <span class="swatch" id="sw-traj"></span> 키프레임 궤적</label>
      <label id="reveal-wrap"><input type="checkbox" id="reveal" checked> <span class="swatch" id="sw-fresh"></span> 관측된 점만 쌓기</label>
      <span class="hint">드래그: 회전 · 휠: 확대 · 우클릭 드래그: 이동</span>
    </div>
    <div id="plot"></div>
  </section>

  <section class="timeline" aria-label="주행 타임라인">
    <div class="scrub">
      <button type="button" id="play">재생</button>
      <div class="track">
        <div class="ticks" id="ticks" aria-hidden="true"></div>
        <input type="range" id="time" min="0" step="1" aria-label="주행 시각">
      </div>
      <div class="clock" id="clock"></div>
    </div>
    <div class="events" id="events"></div>
  </section>

  <section class="notes" id="notes"></section>
</div>

<script src="https://cdnjs.cloudflare.com/ajax/libs/plotly.js/2.27.0/plotly.min.js"></script>
<script id="run-data" type="application/json">__DATA__</script>
<script>
(function () {
  const D = JSON.parse(document.getElementById("run-data").textContent);
  const LABEL = { new_map: "새 맵 시작", map_switch: "맵 병합", loop: "루프 클로저" };
  const COLOR_VAR = { new_map: "--muted", map_switch: "--merge", loop: "--loop" };
  const T = D.traj, N = T.length, U = D.unit === "m" ? "m" : "ORB 단위";
  const fmt = (v, d = 1) => Number(v).toFixed(d);
  const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

  document.getElementById("title").textContent = D.name + " 주행 지도";
  const specs = [
    ["주행 시간", fmt(D.duration, 0), "s"],
    ["주행 거리", fmt(D.pathLength), U],
    ["키프레임", String(N), ""],
    ["맵 포인트", D.nPointsFile.toLocaleString("ko-KR"), ""],
    ["카메라 들림 (추정)", D.elevation === null ? "-" : fmt(D.elevation), "°"],
    ["스케일", (D.scaleApprox ? "≈" : "") + D.scale, "m / ORB 단위"],
  ];
  document.getElementById("specs").innerHTML = specs.map(([k, v, u]) =>
    `<div><dt>${k}</dt><dd>${v} <small>${u}</small></dd></div>`).join("");

  const notes = [
    "좌표는 첫 키프레임의 차 위치를 원점으로, x 전방 · y 왼쪽 · z 위입니다. 카메라가 위로 기울어 있어 궤적에 평면을 맞춰 바닥을 수평으로 돌렸습니다.",
    D.scaleApprox ? "스케일은 이 실행의 확정값이 로그에 남지 않아 이전 주행들의 값으로 근사했습니다. 거리에 몇 % 오차가 있을 수 있습니다." : "스케일은 scale_aligner 가 휠 오도메트리로 확정한 값입니다.",
    "맵 포인트는 키프레임 여러 장에서 삼각측량된 ORB 특징점입니다. 색은 높이를 나타내며, 궤적에서 멀리 떨어진 점은 뺐습니다.",
    D.seenSource === "recorded" ? "‘관측된 점만 쌓기’를 켜면 각 점이 처음 관측된 시각(mono_node 가 기록한 키프레임 시각)에 나타나고, 방금 관측된 점은 주황색으로 잠깐 강조됩니다. 점의 위치는 최종 최적화된 지도 기준입니다."
      : D.seenSource === "estimated" ? "‘관측된 점만 쌓기’를 켜면 각 점이 키프레임 카메라 시야에 처음 들어온 시각에 나타나고, 방금 관측된 점은 주황색으로 잠깐 강조됩니다. 이 실행은 관측 시각이 기록되기 전이라 카메라 포즈와 내부 파라미터로 추정한 시각입니다 (가림은 고려하지 않음)."
      : "",
    `생성: <code>ros2 run orb_slam plot_map --html</code>`,
  ];
  document.getElementById("notes").innerHTML = notes.filter(Boolean).map((n) => `<p>${n}</p>`).join("");
  if (!D.seenSource) document.getElementById("reveal-wrap").hidden = true;

  // 타임라인
  const slider = document.getElementById("time");
  slider.max = String(N - 1);
  slider.value = String(N - 1);
  const ticks = document.getElementById("ticks");
  const evBox = document.getElementById("events");
  D.events.forEach((e) => {
    const tick = document.createElement("span");
    tick.style.left = (100 * e.t / D.duration) + "%";
    tick.dataset.kind = e.kind;
    tick.title = `${LABEL[e.kind]} ${fmt(e.t)} s`;
    ticks.appendChild(tick);
    const b = document.createElement("button");
    b.type = "button";
    b.dataset.kind = e.kind;
    b.innerHTML = `<span class="dot"></span>${LABEL[e.kind]} <span class="t">${fmt(e.t)} s</span>`;
    b.addEventListener("click", () => { stop(); setFrame(e.k); });
    evBox.appendChild(b);
  });

  const clock = document.getElementById("clock");
  const plotEl = document.getElementById("plot");
  const hasPlotly = typeof window.Plotly !== "undefined";

  function colors() {
    return { bg: css("--panel"), ink: css("--ink"), muted: css("--muted"), line: css("--line"),
      accent: css("--accent"), low: css("--pt-low"), high: css("--pt-high"),
      merge: css("--merge"), loop: css("--loop"), fresh: css("--fresh") };
  }

  function paintChrome() {
    const c = colors();
    document.getElementById("sw-points").style.background = `linear-gradient(90deg, ${c.low}, ${c.high})`;
    document.getElementById("sw-traj").style.background = c.accent;
    document.getElementById("sw-fresh").style.background = c.fresh;
    ticks.querySelectorAll("span").forEach((s) => { s.style.background = css(COLOR_VAR[s.dataset.kind]); });
    evBox.querySelectorAll("button").forEach((b) => { b.querySelector(".dot").style.background = css(COLOR_VAR[b.dataset.kind]); });
  }

  const P = D.points;  // [x, y, z, 처음 관측된 시각] — 시각 순으로 정렬돼 있다
  const col = (rows, i) => rows.map((r) => r[i]);
  const zs = col(P, 2);
  const zMin = Math.min(...zs), zMax = Math.max(...zs);
  const FRESH = D.duration / 20 * 0.75;  // 재생 0.75 초 동안 '방금 관측' 으로 강조
  // 시각 t 까지 관측된 점의 개수 (P 가 시각 순이라 이분 탐색)
  function seenCount(t) {
    let lo = 0, hi = P.length;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (P[mid][3] <= t) lo = mid + 1; else hi = mid; }
    return lo;
  }
  const revealOn = () => Boolean(D.seenSource) && document.getElementById("reveal").checked;
  function traces(k) {
    const c = colors();
    const upto = T.slice(0, k + 1);
    const ev = D.events.filter((e) => e.k <= k);
    const showPts = document.getElementById("show-points").checked;
    const n = revealOn() ? seenCount(T[k][0]) : P.length;
    const m = revealOn() && k < N - 1 ? seenCount(T[k][0] - FRESH) : n;
    const old = P.slice(0, m), fresh = P.slice(m, n);
    return [
      { type: "scatter3d", mode: "markers", x: col(old, 0), y: col(old, 1), z: col(old, 2), hoverinfo: "skip",
        marker: { size: 1.2, color: col(old, 2), cmin: zMin, cmax: zMax,
          colorscale: [[0, c.low], [1, c.high]], opacity: 0.6 },
        visible: showPts },
      { type: "scatter3d", mode: "markers", x: col(fresh, 0), y: col(fresh, 1), z: col(fresh, 2),
        hoverinfo: "skip", marker: { size: 1.8, color: c.fresh, opacity: 0.9 }, visible: showPts },
      { type: "scatter3d", mode: "lines", x: col(upto, 1), y: col(upto, 2), z: col(upto, 3),
        line: { color: c.accent, width: 5 }, hoverinfo: "skip",
        visible: document.getElementById("show-traj").checked },
      { type: "scatter3d", mode: "markers", x: [T[k][1]], y: [T[k][2]], z: [T[k][3]],
        marker: { size: 6, color: c.ink, symbol: "diamond" },
        hovertemplate: `t = ${fmt(T[k][0])} s<extra></extra>` },
      { type: "scatter3d", mode: "markers+text", x: [T[0][1]], y: [T[0][2]], z: [T[0][3]],
        marker: { size: 5, color: c.bg, line: { color: c.ink, width: 2 } }, text: ["출발"],
        textposition: "top center", textfont: { color: c.ink, size: 12 }, hoverinfo: "skip" },
      { type: "scatter3d", mode: "markers", x: ev.map((e) => T[e.k][1]), y: ev.map((e) => T[e.k][2]),
        z: ev.map((e) => T[e.k][3]), text: ev.map((e) => `${LABEL[e.kind]} ${fmt(e.t)} s`),
        marker: { size: 7, line: { color: c.ink, width: 1 },
          color: ev.map((e) => (e.kind === "loop" ? c.loop : e.kind === "map_switch" ? c.merge : c.muted)) },
        hovertemplate: "%{text}<extra></extra>" },
    ];
  }

  function layout() {
    const c = colors();
    const axis = (title) => ({ title: { text: title, font: { color: c.muted, size: 12 } },
      color: c.muted, gridcolor: c.line, zerolinecolor: c.line, showbackground: false,
      tickfont: { color: c.muted, size: 10 } });
    return { paper_bgcolor: c.bg, plot_bgcolor: c.bg, margin: { l: 0, r: 0, t: 0, b: 0 },
      showlegend: false, font: { family: "IBM Plex Sans KR, sans-serif", color: c.ink },
      scene: { aspectmode: "data", xaxis: axis(`x 전방 [${U}]`), yaxis: axis(`y 왼쪽 [${U}]`),
        zaxis: axis(`z 위 [${U}]`), camera: { eye: { x: -0.9, y: -1.5, z: 1.1 } } },
      uirevision: "keep" };
  }

  let frame = N - 1;
  function setFrame(k) {
    frame = Math.max(0, Math.min(N - 1, k | 0));
    slider.value = String(frame);
    const dist = T.slice(1, frame + 1).reduce((s, r, i) => s + Math.hypot(r[1] - T[i][1], r[2] - T[i][2], r[3] - T[i][3]), 0);
    const nPts = revealOn() ? seenCount(T[frame][0]) : P.length;
    clock.textContent = `t = ${fmt(T[frame][0])} / ${fmt(D.duration)} s · ${fmt(dist)} ${U} · 점 ${nPts.toLocaleString("ko-KR")}`;
    if (hasPlotly) Plotly.react(plotEl, traces(frame), layout(), { responsive: true, displaylogo: false });
  }

  let timer = null;
  const playBtn = document.getElementById("play");
  function stop() { if (timer) { cancelAnimationFrame(timer); timer = null; } playBtn.textContent = "재생"; }
  playBtn.addEventListener("click", () => {
    if (timer) { stop(); return; }
    if (frame >= N - 1) setFrame(0);
    playBtn.textContent = "멈춤";
    const total = 20000; // 주행 전체를 약 20 초에 재생
    let last = performance.now(), acc = 0;
    const step = (now) => {
      acc += (now - last) / total * D.duration; last = now;
      let k = frame;
      while (k < N - 1 && T[k + 1][0] <= T[frame][0] + acc) k++;
      if (k !== frame) { acc -= T[k][0] - T[frame][0]; setFrame(k); }
      if (frame >= N - 1) { stop(); return; }
      timer = requestAnimationFrame(step);
    };
    timer = requestAnimationFrame(step);
  });
  slider.addEventListener("input", () => { stop(); setFrame(Number(slider.value)); });
  ["show-points", "show-traj", "reveal"].forEach((id) =>
    document.getElementById(id).addEventListener("change", () => setFrame(frame)));

  function retheme() { paintChrome(); setFrame(frame); }
  window.matchMedia("(prefers-color-scheme: dark)").addEventListener("change", retheme);
  new MutationObserver(retheme).observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

  if (!hasPlotly) {
    plotEl.innerHTML = '<p class="fallback">3D 뷰어 라이브러리(Plotly)를 불러오지 못했습니다. 인터넷 연결을 확인한 뒤 새로고침해 주세요. 위의 수치와 아래 타임라인은 그대로 볼 수 있습니다.</p>';
  }
  paintChrome();
  setFrame(N - 1);
})();
</script>
"""


def main():
    ap = argparse.ArgumentParser(description="ORB-SLAM3 키프레임 궤적 + 맵 포인트 시각화")
    ap.add_argument("--trajectory", default=str(ORB_DIR / "kf_trajectory.txt"))
    ap.add_argument("--points", default=str(ORB_DIR / "map_points.ply"),
                    help="없으면 궤적만 그린다")
    ap.add_argument("--events", help="그 실행의 launch.log (새 맵 / 맵 병합 / 루프 클로저 표시)")
    ap.add_argument("--name", help="그림 제목에 쓸 실행 이름 (기본: 궤적 파일 이름)")
    ap.add_argument("--scale", type=float,
                    help="1 ORB 단위 = scale m. 주지 않으면 --scale-file 에서 읽는다")
    ap.add_argument("--scale-file",
                    help="scale_aligner 가 저장한 scale.yaml (기본: 궤적 파일과 같은 폴더)")
    ap.add_argument("--scale-approx", action="store_true",
                    help="스케일이 그 실행의 확정값이 아니라 근사값임을 그림에 표시한다")
    ap.add_argument("--level", choices=["pca", "mount", "none"], default="pca")
    ap.add_argument("--pitch-deg", type=float, default=0.0,
                    help="--level mount 에서 쓸 카메라 들림 각도 (위로 +)")
    ap.add_argument("--min-obs", type=int, default=2,
                    help="이 수 이상의 키프레임에서 관측된 맵 포인트만 그린다")
    ap.add_argument("--zrange", type=float, nargs=2, metavar=("ZMIN", "ZMAX"),
                    help="이 높이 범위의 맵 포인트만 위에서 본 그림에 그린다 (정렬 후 단위)")
    ap.add_argument("--min-height", type=float,
                    help="이 높이 이상인 맵 포인트만 모든 출력에 그린다 "
                         "(정렬 후 z, 0 = 출발 시점 카메라 높이)")
    ap.add_argument("--max-height", type=float,
                    help="이 높이 이하인 맵 포인트만 모든 출력에 그린다 (--min-height 와 함께 쓸 수 있다)")
    ap.add_argument("--out", help="정적 그림 저장 경로 (.png / .pdf)")
    ap.add_argument("--animate", help="주행 애니메이션 저장 경로 (.gif)")
    ap.add_argument("--speed", type=float, default=20.0, help="애니메이션 재생 배속")
    ap.add_argument("--fps", type=int, default=12, help="애니메이션 초당 프레임")
    ap.add_argument("--html", help="3D 뷰어 HTML 저장 경로")
    ap.add_argument("--static-points", action="store_true",
                    help="애니메이션에서 맵 포인트를 처음부터 다 보인다 (관측 시각대로 쌓지 않는다)")
    ap.add_argument("--settings",
                    help="관측 시각 추정에 쓸 ORB-SLAM3 설정 yaml (기본: orb_slam 의 imx219_820x616.yaml)")
    ap.add_argument("--reveal-range", type=float, default=3.0,
                    help="관측 시각 추정: 점이 보인 가장 가까운 거리의 이 배수까지만 보인 것으로 친다")
    ap.add_argument("--outdir",
                    help="출력 경로를 하나도 주지 않았을 때 세 가지를 저장할 폴더 (기본: 궤적 파일 폴더)")
    ap.add_argument("--show", action="store_true",
                    help="파일 대신 정적 그림을 창으로 띄운다")
    args = ap.parse_args()
    if args.scale is None:
        scale_file = args.scale_file or Path(args.trajectory).parent / "scale.yaml"
        args.scale = load_scale(scale_file, args.trajectory) or 1.0

    run = Run(args)
    run.summary()
    if args.show:
        plot_static(run, None, args.zrange)
    elif not (args.out or args.animate or args.html):
        # 출력 경로를 하나도 안 주면 세 가지를 모두 <이름>_map.png / _run.gif / _viewer.html 로
        outdir = Path(args.outdir or Path(args.trajectory).parent).expanduser()
        outdir.mkdir(parents=True, exist_ok=True)
        args.out = str(outdir / f"{run.name}_map.png")
        args.animate = str(outdir / f"{run.name}_run.gif")
        args.html = str(outdir / f"{run.name}_viewer.html")
    if args.out:
        plot_static(run, args.out, args.zrange)
    if args.animate:
        animate(run, args.animate, args.speed, args.fps, reveal=not args.static_points)
    if args.html:
        export_html(run, args.html)


if __name__ == "__main__":
    main()
