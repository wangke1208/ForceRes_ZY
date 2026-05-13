#!/usr/bin/env bash
# 打印 example/lib/libforce_res.a 的逻辑大小（字节）与人可读大小。
# 建议在容器 sy_dev 内、挂载仓库根下执行（与编库环境一致）。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LIB="${ROOT}/example/lib/libforce_res.a"
if [[ ! -f "${LIB}" ]]; then
  echo "未找到 ${LIB}，请先执行 ./scripts/menu.sh 选 1（或 ./scripts/build_and_sync_example.sh / 手动 cmake）。" >&2
  exit 1
fi
if stat --version >/dev/null 2>&1; then
  BYTES=$(stat -c%s "${LIB}")
else
  BYTES=$(stat -f%z "${LIB}")
fi
echo "libforce_res.a bytes: ${BYTES}"
ls -lh "${LIB}"
