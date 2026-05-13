#!/usr/bin/env bash
# 在「完整 ForceRes_ZY 仓库根」旁执行本脚本所在目录为 example/ 时，
# 将编库产物与第三方头复制进 example/，使 example/ 可单独打包复制。
set -euo pipefail
EXAMPLE_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${EXAMPLE_DIR}/.." && pwd)"

mkdir -p "${EXAMPLE_DIR}/3rd" "${EXAMPLE_DIR}/lib" "${EXAMPLE_DIR}/include"

cp -f "${REPO_ROOT}/rokae_header/rokae_force_controller_public.hpp" "${EXAMPLE_DIR}/include/"

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
  echo "warning: 未找到 ${LIB_SRC}，请先在仓库根执行 cmake 构建主静态库。" >&2
fi

echo "sync_from_repo.sh 完成。可在 ${EXAMPLE_DIR} 下执行: cmake -S . -B build && cmake --build build"
