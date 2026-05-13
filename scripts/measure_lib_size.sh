#!/usr/bin/env bash
# 打印 example/lib/libforce_res.a 的逻辑大小（字节）与人可读大小。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
LIB="${ROOT}/example/lib/libforce_res.a"
if [[ ! -f "${LIB}" ]]; then
  echo "未找到 ${LIB}，请先在仓库根构建主静态库。" >&2
  exit 1
fi
if stat --version >/dev/null 2>&1; then
  BYTES=$(stat -c%s "${LIB}")
else
  BYTES=$(stat -f%z "${LIB}")
fi
echo "libforce_res.a bytes: ${BYTES}"
ls -lh "${LIB}"
