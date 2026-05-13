# scripts 说明

均在 **Docker 容器 `sy_dev`** 内、仓库根（如 **`/workspace/ForceRes_ZY`**）执行；勿在宿主机对本机路径混用 `build/`。

## 主入口（推荐）

**[`menu.sh`](menu.sh)**：交互选功能；也支持直接带参数一次执行：

```bash
cd /workspace/ForceRes_ZY
chmod +x scripts/menu.sh   # 首次
./scripts/menu.sh          # 交互，按提示输入 1～4 或 q
./scripts/menu.sh 1        # 等价于选 1，非交互
```

| 序号 | 作用 |
|------|------|
| 1 | 调用 `build_and_sync_example.sh`：CMake 编 `libforce_res.a`，**成功后再**填充 `example/` |
| 2 | 调用 `sync_example_from_repo.sh`：只拷头文件与 `3rd/` 等进 `example/` |
| 3 | 调用 `measure_lib_size.sh`：打印 `example/lib/libforce_res.a` 大小 |
| 4 | 调用 `sync_example_to_workspace_test.sh`：`rsync` 将 `example/` 增量同步到测试目录（默认 `/workspace/Test` 等，见脚本内说明） |

## 子脚本（可单独调用）

| 文件 | 说明 |
|------|------|
| `build_and_sync_example.sh` | 编库 + 成功后同步；环境变量见文件头注释 |
| `sync_example_from_repo.sh` | 仅同步 |
| `measure_lib_size.sh` | 仅测 `.a` 体积 |
| `sync_example_to_workspace_test.sh` | 仅 rsync 到外部目录；可设 `EXAMPLE_SYNC_DEST` |

详见各脚本文件头注释。
