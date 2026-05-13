#!/usr/bin/env bash
# 在 Docker 容器 sy_dev 内、挂载后的仓库根执行（验证路径为 /workspace/ForceRes_ZY）。
# 配置并编译 libforce_res.a；仅当编译成功时才将产物与依赖同步到 example/。
# 也可由 scripts/menu.sh 选 1 调用本脚本。
# 勿在宿主机对本机副本路径单独跑本脚本：若 build/ 在容器内生成，与本机路径混用会触发 CMake 缓存路径冲突。
# 可选环境变量：BUILD_DIR（默认 build）、CMAKE_BUILD_TYPE（默认 Release）、CMAKE_GENERATOR（若未设且存在 ninja 则用 Ninja）、CMAKE_EXTRA_ARGS（附加 cmake 参数，按 shell 分词展开）。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-build}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

cd "${REPO_ROOT}"
REPO_ROOT_REAL="$(pwd -P 2>/dev/null || pwd)"
if [[ -f "${BUILD_DIR}/CMakeCache.txt" ]]; then
  cached_home="$(grep -m1 '^CMAKE_HOME_DIRECTORY:INTERNAL=' "${BUILD_DIR}/CMakeCache.txt" 2>/dev/null | sed 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//' || true)"
  if [[ -n "${cached_home}" && "${cached_home}" != "${REPO_ROOT_REAL}" ]]; then
    echo "error: ${BUILD_DIR}/CMakeCache.txt 中的源码路径为「${cached_home}」，与当前目录「${REPO_ROOT_REAL}」不一致。" >&2
    echo "      本工程约定：编库与同步均在容器 sy_dev 内、挂载工程根（一般为 /workspace/ForceRes_ZY）执行。" >&2
    echo "      请勿在宿主机路径对同一仓库跑 cmake/本脚本。若确需在容器内重建，可进入容器后删除 ${BUILD_DIR} 再执行。" >&2
    exit 1
  fi
fi

CMAKE_ARGS=(-S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}")
if [[ -n "${CMAKE_GENERATOR:-}" ]]; then
  CMAKE_ARGS+=(-G "${CMAKE_GENERATOR}")
elif command -v ninja >/dev/null 2>&1; then
  CMAKE_ARGS+=(-G Ninja)
fi

echo "==> CMake 配置: ${CMAKE_ARGS[*]} ${CMAKE_EXTRA_ARGS:-}"
# shellcheck disable=SC2086
cmake "${CMAKE_ARGS[@]}" ${CMAKE_EXTRA_ARGS:-}

JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc 2>/dev/null || echo 4)"
echo "==> 编译 force_res (-j${JOBS})"
cmake --build "${BUILD_DIR}" -j"${JOBS}"

echo "==> 编译成功，同步到 example/"
exec "${SCRIPT_DIR}/sync_example_from_repo.sh"
