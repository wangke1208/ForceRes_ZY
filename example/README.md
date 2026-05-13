# 自包含 SDK 示例包（可单独复制）

本目录模拟「客户仅拿到 `lib/` + `include/` + `3rd/` + `demos/`」时的布局：**不依赖**上级仓库的 `rokae_src`、`rokae_header`（除同步脚本从开发树拉取文件外）。

## 在开发机上准备（位于完整仓库内时）

1. 在**仓库根**构建主静态库（推荐 **Release** 以减小 `libforce_res.a` 体积）：

```bash
cd /path/to/ForceRes_ZY
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

生成 `example/lib/libforce_res.a`（与根 `CMakeLists.txt` 一致）。在 **GCC** 下编 `example` 内 demo 时，会为可执行文件加 **`-Wl,--gc-sections`**，进一步减小最终二进制体积。
2. 执行同步脚本，把瘦头、Eigen、spdlog、`.a` 拷入本目录：

```bash
chmod +x example/sync_from_repo.sh   # 首次
./example/sync_from_repo.sh
```

3. **仅在本目录**配置并编译所有 demo：

```bash
cd example
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
# 可执行文件: build/joint_drag_test 等
```

要求：与编 `libforce_res.a` 时相同量级的 **GCC**、**CMake 3.20+**、系统 **Boost**、**pthread**。**静态库与可执行文件须为同一目标架构**（例如均为 `aarch64` Linux）。

## 复制到其它机器

将 **`example/` 整目录**（须已含 `sync_from_repo.sh` 跑完后的 `3rd/eigen`、`3rd/spdlog`、`include/rokae_force_controller_public.hpp`、`lib/libforce_res.a`、`demos/`）打包复制；在 **相同或兼容的 Linux + GCC + Boost** 下执行上节步骤 3 即可。

**不要**只复制空壳：至少要有 `3rd/`、`lib/*.a`、`include/` 下的瘦头。

## 目录说明

| 路径 | 作用 |
|------|------|
| `include/` | `rokae_force_controller_public.hpp`（同步自 `rokae_header/`） |
| `lib/` | `libforce_res.a`（同步自仓库根构建产物） |
| `3rd/eigen`、`3rd/spdlog` | 编译 demo 所需头（与主工程版本一致） |
| `demos/` | 各 `main` 与 `client_demo_macros.h`（日志宏，不随商业 SDK 交付时可删） |

更细的对外 API 说明见 **`include/README.md`**（若存在）。
