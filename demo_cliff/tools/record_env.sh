#!/usr/bin/env bash
# 이 컴퓨터의 시뮬레이션 환경을 기록한다. GPU·CPU·드라이버가 다르면 Gazebo 렌더링과 통신 타이밍이 달라져
# 결과가 조금씩 달라질 수 있어서, 다른 기계 결과를 비교·합칠 때 근거로 쓴다.
#   tools/record_env.sh                 # 화면에
#   tools/record_env.sh 파일경로        # 파일로
[ -n "$1" ] && exec > "$1"      # 파일 이름을 주면 그 파일로
{
  echo "기록 시각: $(date '+%F %T')"
  echo "호스트: $(hostname)"
  echo "OS: $(. /etc/os-release 2>/dev/null && echo "$PRETTY_NAME")"
  echo "커널: $(uname -r)"
  echo "CPU: $(lscpu 2>/dev/null | awk -F: '/Model name/ {gsub(/^ +/, "", $2); print $2; exit}') ($(nproc)코어)"
  echo "메모리: $(free -h 2>/dev/null | awk '/^Mem:|^메모리:/ {print $2; exit}')"
  echo "GPU: $(lspci 2>/dev/null | grep -i -E 'vga|3d controller' | sed -E 's/^[0-9a-f:.]+ //' | paste -sd ';' -)"
  if command -v nvidia-smi >/dev/null 2>&1; then
    echo "NVIDIA: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>/dev/null | paste -sd ';' -)"
  fi
  if command -v glxinfo >/dev/null 2>&1; then
    echo "OpenGL: $(glxinfo -B 2>/dev/null | awk -F: '/OpenGL renderer string/ {gsub(/^ +/, "", $2); print $2}')"
  fi
  echo "Gazebo: $(gzserver --version 2>/dev/null | grep -o -m1 'version [0-9.]*')"
  echo "RMW: ${RMW_IMPLEMENTATION:-기본(Fast DDS)}"
  dpkg -l 2>/dev/null | awk '/^ii/ && $2 ~ /^ros-humble-(navigation2|nav2-bringup|slam-toolbox|gazebo-ros-pkgs|rmw-cyclonedds-cpp)$/ {print $2 ": " $3}'
}
