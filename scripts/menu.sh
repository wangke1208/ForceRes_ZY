#!/usr/bin/env bash
# 仓库脚本菜单：请在容器 sy_dev 内、仓库根挂载路径（如 /workspace/ForceRes_ZY）下执行。
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

die() { echo "error: $*" >&2; exit 1; }

pick() {
  case "${1:-}" in
    1) bash "${SCRIPT_DIR}/build_and_sync_example.sh" ;;
    2) bash "${SCRIPT_DIR}/sync_example_from_repo.sh" ;;
    3) bash "${SCRIPT_DIR}/measure_lib_size.sh" ;;
    4) bash "${SCRIPT_DIR}/sync_example_to_workspace_test.sh" ;;
    q|Q) exit 0 ;;
    *) return 1 ;;
  esac
}

show_menu() {
  echo ""
  echo "  1  编库并同步 example（成功才同步）"
  echo "  2  仅同步 example（头与 3rd，.a 视情况复制）"
  echo "  3  查看 libforce_res.a 体积"
  echo "  4  rsync 将 example 拷到测试目录"
  echo "  q  退出"
  echo ""
}

if [[ $# -ge 1 ]]; then
  case "$1" in
    1 | 2 | 3 | 4 | q | Q)
      pick "$1"
      exit $?
      ;;
    *)
      die "无效参数，请传 1 / 2 / 3 / 4 / q，或不传参进入交互"
      ;;
  esac
fi

cd "${REPO_ROOT}" || die "无法进入 ${REPO_ROOT}"
while true; do
  show_menu
  read -r -p "输入序号: " c || exit 0
  [[ -z "${c// /}" ]] && exit 0
  if ! pick "${c}"; then
    echo "无效选项，请输入 1～4 或 q" >&2
  fi
done
