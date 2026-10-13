#!/usr/bin/env python3
"""mono_node 가 종료 시 저장한 키프레임 궤적과 맵 포인트를 그린다.

입력 (둘 다 같은 맵, orb_world 프레임, 스케일 없는 ORB 단위)
    --trajectory  키프레임 궤적, TUM 형식: t tx ty tz qx qy qz qw  (T_world_cam)
    --points      맵 포인트, ASCII PLY: x y z [observations]

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

수평 정렬 후 좌표는 첫 키프레임의 로봇 기준 x 전방, y 왼쪽, z 위이고, --scale 을 주면
(scale_aligner 가 출력한 "1 ORB 단위 = s m") 미터가 된다.

    ros2 run orb_slam plot_map --scale 2.39
    ros2 run orb_slam plot_map --level mount --pitch-deg 17.5 --out map.png
"""

import argparse
import math
from pathlib import Path

import numpy as np

ORB_DIR = Path("~/.ros/orbslam3").expanduser()


def load_trajectory(path):
    data = np.loadtxt(path, ndmin=2)
    if data.size == 0:
        raise SystemExit(
            f"{path} 가 비어 있다. 종료 시점의 현재 맵에 키프레임이 없었다 (막 새로 만든 맵)."
        )
    return data[:, 0], data[:, 1:4]


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


def main():
    ap = argparse.ArgumentParser(description="ORB-SLAM3 키프레임 궤적 + 맵 포인트 시각화")
    ap.add_argument("--trajectory", default=str(ORB_DIR / "kf_trajectory.txt"))
    ap.add_argument("--points", default=str(ORB_DIR / "map_points.ply"),
                    help="없으면 궤적만 그린다")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="1 ORB 단위 = scale m (scale_aligner 의 '스케일 확정' 값)")
    ap.add_argument("--level", choices=["pca", "mount", "none"], default="pca")
    ap.add_argument("--pitch-deg", type=float, default=0.0,
                    help="--level mount 에서 쓸 카메라 들림 각도 (위로 +)")
    ap.add_argument("--min-obs", type=int, default=2,
                    help="이 수 이상의 키프레임에서 관측된 맵 포인트만 그린다")
    ap.add_argument("--zrange", type=float, nargs=2, metavar=("ZMIN", "ZMAX"),
                    help="이 높이 범위의 맵 포인트만 위에서 본 그림에 그린다 (정렬 후 단위)")
    ap.add_argument("--out", help="그림 저장 경로 (.png). 없으면 창으로 띄운다")
    args = ap.parse_args()

    stamps, traj = load_trajectory(args.trajectory)

    pca_up, spread = fit_up(traj) if len(traj) >= 3 else (None, 0.0)
    level = args.level
    if level == "pca" and (pca_up is None or spread < 0.05):
        print("궤적이 거의 직선이라 바닥 평면을 정할 수 없다 -> --level mount 로 그린다")
        level = "mount"
    if level == "pca":
        axes = axes_from_up(pca_up)
    elif level == "mount":
        axes = mount_axes(args.pitch_deg)
    else:
        axes = mount_axes(0.0)

    origin = traj[0]
    traj_l = (traj - origin) @ axes.T * args.scale

    pts_l, obs = None, None
    if Path(args.points).is_file():
        ply = load_ply(args.points)
        pts = np.column_stack([ply["x"], ply["y"], ply["z"]]) if ply else np.empty((0, 3))
        obs = ply.get("observations", np.full(len(pts), args.min_obs))
        keep = obs >= args.min_obs
        pts_l = (pts[keep] - origin) @ axes.T * args.scale
        obs = obs[keep]
        print(f"맵 포인트 {len(pts)} 개 중 관측 {args.min_obs} 회 이상 {len(pts_l)} 개")
    else:
        print(f"맵 포인트 파일이 없어 궤적만 그린다: {args.points}")

    unit = "m" if args.scale != 1.0 else "ORB units"
    steps = np.linalg.norm(np.diff(traj_l, axis=0), axis=1)
    print(f"키프레임 {len(traj_l)} 개, {stamps[-1] - stamps[0]:.1f} s, "
          f"경로 길이 {steps.sum():.2f} {unit}")
    print(f"수평 정렬: {level}"
          + (f" (pitch {args.pitch_deg:.1f} deg)" if level == "mount" else ""))
    if pca_up is not None:
        print(f"궤적 평면으로 추정한 카메라 들림 각도 {elevation_deg(pca_up):.1f} deg, "
              f"좌우 기울기 {math.degrees(math.asin(pca_up[0])):.1f} deg "
              f"(평면 신뢰도 {spread:.2f}, 0.05 미만이면 불확실)")
    print(f"정렬 후 키프레임 높이: 표준편차 {traj_l[:, 2].std():.3f} {unit}, "
          f"범위 {np.ptp(traj_l[:, 2]):.3f} {unit}  (평지 주행이면 0 에 가까워야 한다)")

    import matplotlib
    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (top, side) = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": [3, 2]})
    t = stamps - stamps[0]

    if pts_l is not None and len(pts_l):
        top_pts = pts_l
        if args.zrange:
            zmin, zmax = args.zrange
            top_pts = pts_l[(pts_l[:, 2] >= zmin) & (pts_l[:, 2] <= zmax)]
        top.scatter(top_pts[:, 0], top_pts[:, 1], s=1, c="0.6", label="map points")
        side.scatter(pts_l[:, 0], pts_l[:, 2], s=1, c="0.6", label="map points")

    for ax, (i, j) in ((top, (0, 1)), (side, (0, 2))):
        ax.plot(traj_l[:, i], traj_l[:, j], "-", c="0.3", lw=0.8)
        sc = ax.scatter(traj_l[:, i], traj_l[:, j], c=t, cmap="viridis", s=12, zorder=3,
                        label="keyframes")
        ax.plot(*traj_l[0, [i, j]], "^", c="tab:green", ms=10, zorder=4, label="start")
        ax.plot(*traj_l[-1, [i, j]], "s", c="tab:red", ms=9, zorder=4, label="end")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True, lw=0.3)

    top.set_title("Top view (x forward, y left at first keyframe)")
    top.set_xlabel(f"x [{unit}]")
    top.set_ylabel(f"y [{unit}]")
    top.legend(loc="best", fontsize=8)
    side.set_title("Side view (height should stay flat for keyframes)")
    side.set_xlabel(f"x [{unit}]")
    side.set_ylabel(f"z [{unit}]")
    fig.colorbar(sc, ax=side, label="time since first keyframe [s]")
    fig.suptitle(f"{Path(args.trajectory).name}  |  level={level}  |  scale={args.scale:g}")
    fig.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f"저장: {args.out}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
