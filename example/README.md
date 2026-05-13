# 自包含 SDK 示例包（可单独复制）

本目录模拟「客户仅拿到 `lib/` + `include/` + `3rd/` + `demos/`」时的布局：**不依赖**上级仓库的 `rokae_src`、`rokae_header`（由仓库根下 **`scripts/menu.sh`** 在**容器 `sy_dev` 内**从开发树拉取文件）。

## 在容器 `sy_dev` 内准备（与主工程编库同一环境）

**约定**：编主库、同步 `example/`、以及编本目录下 demo，均在 **Docker 容器 `sy_dev`** 中、挂载后的仓库根（验证路径 **`/workspace/ForceRes_ZY`**）进行；勿在宿主机对本机路径单独跑 **`./scripts/menu.sh`** 或 `cmake`，以免与容器内 `build/` 缓存冲突。

**推荐：在容器内仓库根执行 `./scripts/menu.sh`，选 1**（编库成功后再填充本目录）：

```bash
cd /workspace/ForceRes_ZY
chmod +x scripts/menu.sh   # 首次
./scripts/menu.sh          # 输入 1
```

生成 `example/lib/libforce_res.a`，并填充 `include/rokae_force_controller.hpp`、`3rd/eigen`、`3rd/spdlog`。在 **GCC** 下编 `example` 内 demo 时，会为可执行文件加 **`-Wl,--gc-sections`**，进一步减小最终二进制体积。

若已在容器内编过主库，只需刷新 `example/` 时，可 **`./scripts/menu.sh` 选 2** 或：

```bash
cd /workspace/ForceRes_ZY
./scripts/sync_example_from_repo.sh
```

**仅在 `example/` 内**配置并编译所有 demo（仍在容器内）：

```bash
cd /workspace/ForceRes_ZY/example
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
# 可执行文件在 build/demos/ 下，例如 ./build/demos/joint_drag_test
```

要求：与编 `libforce_res.a` 时相同量级的 **GCC**、**CMake 3.20+**、系统 **Boost**、**pthread**。**静态库与可执行文件须为同一目标架构**（例如均为 `aarch64` Linux）。

更多脚本说明见 **`../scripts/README.md`**。

## 复制到其它机器

将 **`example/` 整目录**（须已含 `3rd/eigen`、`3rd/spdlog`、`include/rokae_force_controller.hpp`、`lib/libforce_res.a`、`demos/`）打包复制；在 **相同或兼容的 Linux + GCC + Boost** 下执行上节「仅在 example 内」的 `cmake` 步骤即可。

**不要**只复制空壳：至少要有 `3rd/`、`lib/*.a`、`include/` 下的 `rokae_force_controller.hpp`。

## 目录说明

| 路径 | 作用 |
|------|------|
| `include/` | `rokae_force_controller.hpp`（由 **`./scripts/menu.sh` 选 2** 或 `sync_example_from_repo.sh` 复制） |
| `lib/` | `libforce_res.a`（同步自仓库根构建产物） |
| `3rd/eigen`、`3rd/spdlog` | 编译 demo 所需头（与主工程版本一致） |
| `demos/` | 各 `main` 与 `client_demo_macros.h`（日志宏，不随商业 SDK 交付时可删） |

更细的对外 API 说明见 **`include/README.md`**（若存在）。
