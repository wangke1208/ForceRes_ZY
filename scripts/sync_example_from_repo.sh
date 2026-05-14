#!/usr/bin/env bash
# 在 Docker 容器 sy_dev 内、挂载后的仓库根执行（与 build_and_sync_example.sh 相同约定，验证路径为 /workspace/ForceRes_ZY）。
# 将主工程头文件与 3rd 依赖复制进 example/，使 example/ 可单独打包构建 demo。
# 通常由 build_and_sync_example.sh 或 menu.sh 选 1 在编译成功后调用；也可在容器内已手动编出 libforce_res.a 后单独执行本脚本。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
EXAMPLE_DIR="${REPO_ROOT}/example"

mkdir -p "${EXAMPLE_DIR}/3rd" "${EXAMPLE_DIR}/lib" "${EXAMPLE_DIR}/include"

cp -f "${REPO_ROOT}/rokae_header/rokae_force_controller.hpp" "${EXAMPLE_DIR}/include/"

if [[ -d "${REPO_ROOT}/3rd/eigen" ]]; then
  rm -rf "${EXAMPLE_DIR}/3rd/eigen"
  cp -R "${REPO_ROOT}/3rd/eigen" "${EXAMPLE_DIR}/3rd/"
else
  echo "error: ${REPO_ROOT}/3rd/eigen 不存在" >&2
  exit 1
fi

if [[ -d "${REPO_ROOT}/3rd/spdlog" ]]; then
  rm -rf "${EXAMPLE_DIR}/3rd/spdlog"
  cp -R "${REPO_ROOT}/3rd/spdlog" "${EXAMPLE_DIR}/3rd/"
else
  echo "error: ${REPO_ROOT}/3rd/spdlog 不存在" >&2
  exit 1
fi

LIB_SRC="${REPO_ROOT}/example/lib/libforce_res.a"
LIB_DST="${EXAMPLE_DIR}/lib/libforce_res.a"
if [[ -f "${LIB_SRC}" ]]; then
  if [[ "$(cd "$(dirname "${LIB_SRC}")" && pwd)/$(basename "${LIB_SRC}")" != "$(cd "$(dirname "${LIB_DST}")" && pwd)/$(basename "${LIB_DST}")" ]]; then
    cp -f "${LIB_SRC}" "${LIB_DST}"
    echo "已复制 libforce_res.a"
  else
    echo "libforce_res.a 已在 example/lib（与仓库构建输出为同一文件，跳过复制）"
  fi
else
  echo "warning: 未找到 ${LIB_SRC}，请先执行 ./scripts/menu.sh 1 或在仓库根构建主静态库。" >&2
fi

echo "完成。下一步可在 ${EXAMPLE_DIR} 下: cmake -S . -B build && cmake --build build"
echo "      demo 可执行文件在 ${EXAMPLE_DIR}/build/demos/（目标列表见 example/demos/CMakeLists.txt）。"
