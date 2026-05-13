#!/usr/bin/env bash
# 将仓库内 example/ 增量同步到目标目录（默认 /workspace/Test）。
# 使用 rsync：只传输变更文件，不每次整目录重写。
#
# 用法（请分两行执行，不要把「# 说明」粘在 chmod 同一行，否则 # 可能被当成 chmod 的参数）：
#   chmod +x scripts/sync_example_to_workspace_test.sh
#   ./scripts/sync_example_to_workspace_test.sh
#
# 指定目标：
#
# 依赖：rsync（容器 sy_env / 常见 Linux 均有）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${SCRIPT_DIR}"
if REPO_ROOT="$(git rev-parse --show-toplevel 2>/dev/null)"; then
  :
else
  REPO_ROOT="$(cd .. && pwd)"
fi

SRC="${REPO_ROOT}/example/"

# 默认目标：若已有共享挂载目录 /workspace（容器常见），则同步到 /workspace/Test；
# 不在容器、且本机无 /workspace 时，退回 $HOME 下目录。
if [[ -n "${EXAMPLE_SYNC_DEST:-}" ]]; then
  DST="${EXAMPLE_SYNC_DEST}"
elif [[ -d /workspace ]]; then
  DST="/workspace/Test"
else
  DST="${HOME}/ForceRes_ZY_example_sync"
  echo "提示: 本机未检测到挂载目录 /workspace，已改用: ${DST}" >&2
  echo "      若在容器内且已挂载共享盘，应能看到 /workspace；也可显式设置 EXAMPLE_SYNC_DEST。" >&2
fi

if [[ ! -d "${SRC}" ]]; then
  echo "error: 源目录不存在: ${SRC}" >&2
  exit 1
fi

if ! command -v rsync >/dev/null 2>&1; then
  echo "error: 未找到 rsync，请先安装。" >&2
  exit 1
fi

mkdir -p "${DST}"

# -a: 归档（权限、时间等），仅传输差异
# 源与目标末尾 /：同步 example 目录「内容」到目标目录内
# 未使用 --delete：目标上多出来的文件保留（纯增量）；若需与 example 完全一致可改为加 --delete
echo "增量同步: ${SRC} -> ${DST}/"
rsync -a "${SRC}" "${DST}/"

echo "完成。"
