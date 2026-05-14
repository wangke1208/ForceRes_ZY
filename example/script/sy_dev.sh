#!/usr/bin/env bash
# 在已运行的 sy_dev 容器内：每次先删除 build/ → 全量配置编译 → 运行指定 demo
# 用法: ./script/sy_dev.sh [1-4]
# 环境变量: CONTAINER_NAME  DOCKER_TEST_DIR

set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-sy_dev}"
DOCKER_TEST_DIR="${DOCKER_TEST_DIR:-/workspace/Test}"

DEMO_TARGETS=(
  joint_drag_test
  joint_drag_test_with_load
)

print_menu() {
  echo "选择要运行的 demo（输入数字）:"
  local i=1
  for t in "${DEMO_TARGETS[@]}"; do
    echo "  $i) $t"
    i=$((i + 1))
  done
}

resolve_choice() {
  local raw="${1:-}"
  raw="${raw//[^0-9]/}"
  if [[ -z "$raw" || "$raw" -lt 1 || "$raw" -gt ${#DEMO_TARGETS[@]} ]]; then
    return 1
  fi
  echo "$raw"
}

if ! docker info >/dev/null 2>&1; then
  echo "错误: 无法连接 Docker，请先启动 Docker Desktop。" >&2
  exit 1
fi

if ! docker container inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null | grep -qx true; then
  echo "错误: 容器「${CONTAINER_NAME}」不存在或未在运行。" >&2
  exit 1
fi

if ! docker exec "${CONTAINER_NAME}" test -d "${DOCKER_TEST_DIR}"; then
  echo "错误: 容器内不存在「${DOCKER_TEST_DIR}」。请检查 DOCKER_TEST_DIR。" >&2
  exit 1
fi

choice=""
if [[ -n "${1:-}" ]]; then
  choice="$(resolve_choice "$1")" || {
    echo "错误: 无效选项「$1」，请输入 1 到 ${#DEMO_TARGETS[@]}。" >&2
    exit 1
  }
elif [[ -t 0 ]]; then
  print_menu
  read -r -p "请输入序号 [1-${#DEMO_TARGETS[@]}]: " user_in
  choice="$(resolve_choice "$user_in")" || {
    echo "错误: 无效输入。" >&2
    exit 1
  }
else
  print_menu >&2
  echo "错误: 非交互环境请传入序号，例如: $0 1" >&2
  exit 1
fi

idx=$((choice - 1))
TARGET="${DEMO_TARGETS[$idx]}"
echo "==> 容器: ${CONTAINER_NAME}  工程: ${DOCKER_TEST_DIR}  目标: ${TARGET}"

docker exec "${CONTAINER_NAME}" bash -lc "set -euo pipefail
cd \"${DOCKER_TEST_DIR}\"
rm -rf build
cmake -S . -B build -G Ninja
cmake --build build -j\"\$(nproc)\" --target \"${TARGET}\"
exec ./build/demos/${TARGET}
"
