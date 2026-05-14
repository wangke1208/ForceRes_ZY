#!/usr/bin/env bash
# 在本机：删除 build/ → 全量 cmake → 只编并运行所选 demo。
#
# 用法（任意当前目录均可）：
#   bash "/path/to/本仓库/script/build&run.sh" [1-2]
#   bash "./script/build&run.sh"          # 交互选 demo
# 文件名含 &，请用 bash 加引号，或执行: './script/build&run.sh'
#
# 仓库根 = 本脚本所在目录的上一级（与文件夹名、路径无关）。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f "${REPO_ROOT}/CMakeLists.txt" ]]; then
  echo "错误: 在「${REPO_ROOT}」未找到 CMakeLists.txt。请保持本文件位于「仓库根/script/build&run.sh」。" >&2
  exit 1
fi

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
  echo "错误: 非交互环境请传入序号，例如: bash \"./script/build&run.sh\" 1" >&2
  exit 1
fi

idx=$((choice - 1))
TARGET="${DEMO_TARGETS[$idx]}"

build_jobs() {
  if [[ -n "${CMAKE_BUILD_PARALLEL_LEVEL:-}" ]]; then
    echo "${CMAKE_BUILD_PARALLEL_LEVEL}"
  elif command -v nproc >/dev/null 2>&1; then
    nproc
  elif command -v getconf >/dev/null 2>&1; then
    getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4
  else
    echo 4
  fi
}

echo "==> 工程: ${REPO_ROOT}  目标: ${TARGET}"
cd "${REPO_ROOT}"
rm -rf build
cmake -S . -B build -G Ninja
cmake --build build -j"$(build_jobs)" --target "${TARGET}"
exec ./build/demos/"${TARGET}"
