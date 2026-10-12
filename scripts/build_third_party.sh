#!/bin/bash
# third_party/ 의 Pangolin 과 ORB-SLAM3(로컬 패치 포함)를 빌드한다. PC 전용 — Pi 는 필요 없다.
#
#   ./scripts/build_third_party.sh          # 증분 빌드
#   JOBS=8 ./scripts/build_third_party.sh
#
# 순서: Pangolin(→ third_party/install) → DBoW2 → g2o → ORB_SLAM3(라이브러리만)
# 그 다음 워크스페이스 루트에서 colcon build --symlink-install.
#
# 주의: ORB_SLAM3/build 에 make clean 이나 --clean-first 를 쓰지 말 것. 그 트리는
# g2o 서브프로젝트를 포함하고 있어 Thirdparty/g2o/lib/libg2o.so 까지 지운다. 깨끗이
# 다시 빌드하려면 해당 build/ 와 lib/ 디렉터리를 지우고 이 스크립트를 다시 돌린다.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TP="$ROOT/third_party"
PREFIX="$TP/install"
JOBS="${JOBS:-$(nproc)}"

# conda 환경이 켜져 있어도 시스템 툴체인/파이썬을 쓰게 한다.
export PATH="/usr/bin:$PATH"

configure() {  # configure <src dir> [cmake args...]
  local src="$1"; shift
  echo "==> $src"
  # 패키지 레지스트리(~/.cmake/packages)를 끈다. 다른 곳에 빌드된 Pangolin 이
  # 잡히지 않고 항상 이 저장소의 third_party/install 을 쓰게 하기 위해서다.
  cmake -S "$src" -B "$src/build" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_FIND_USE_PACKAGE_REGISTRY=OFF -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON "$@"
}

# Pangolin 은 git submodule(v0.6)이다. clone 할 때 --recursive 를 빠뜨렸으면 여기서 받는다.
# Pangolin 자신의 하위 submodule(pybind11)은 BUILD_PANGOLIN_PYTHON=OFF 라 필요 없다.
if [ ! -f "$TP/Pangolin/CMakeLists.txt" ]; then
  echo "==> third_party/Pangolin submodule 초기화"
  git -C "$ROOT" submodule update --init third_party/Pangolin
fi

configure "$TP/Pangolin" -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
  -DBUILD_PANGOLIN_FFMPEG=OFF
cmake --build "$TP/Pangolin/build" -j "$JOBS"
cmake --install "$TP/Pangolin/build"

# ORB_SLAM3 는 DBoW2/g2o 를 타깃이 아닌 lib/*.so 경로로 링크하므로 먼저 따로 빌드한다.
configure "$TP/ORB_SLAM3/Thirdparty/DBoW2"
cmake --build "$TP/ORB_SLAM3/Thirdparty/DBoW2/build" -j "$JOBS"
configure "$TP/ORB_SLAM3/Thirdparty/g2o"
cmake --build "$TP/ORB_SLAM3/Thirdparty/g2o/build" -j "$JOBS"

# Sophus 는 헤더 전용이라 빌드하지 않는다 (ORB_SLAM3 가 include 경로로만 쓴다).

# 예제 실행 파일은 필요 없으므로 라이브러리 타깃만 빌드한다.
configure "$TP/ORB_SLAM3" -DPangolin_DIR="$PREFIX/lib/cmake/Pangolin"
cmake --build "$TP/ORB_SLAM3/build" -j "$JOBS" --target ORB_SLAM3

# mono_node 는 기본으로 ~/.ros/orbslam3/ORBvoc.txt 를 읽는다.
VOC="$HOME/.ros/orbslam3/ORBvoc.txt"
if [ ! -f "$VOC" ]; then
  echo "==> extracting vocabulary to $VOC"
  mkdir -p "$(dirname "$VOC")"
  tar -xzf "$TP/ORB_SLAM3/Vocabulary/ORBvoc.txt.tar.gz" -C "$(dirname "$VOC")"
fi

echo "done: $TP/ORB_SLAM3/lib/libORB_SLAM3.so"
