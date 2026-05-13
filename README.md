# ForceRes_ZY

珞石力控相关 C++ 算法库（`rokae_header` / `rokae_src`）及第三方依赖源码。根目录 CMake **仅**构建静态库 `libforce_res.a`（**不包含** `example/` 下任何源码）。**自包含示例包**见 **`example/`**（复制该目录到别处即可在同环境编 demo，见 `example/README.md`）。

**环境与脚本约定**：主工程编库、填充 `example/`、以及将 `example/` 同步到共享目录等操作，**一律在 Docker 容器 `sy_dev` 内**完成（工作副本为挂载进容器的仓库根，验证路径为 **`/workspace/ForceRes_ZY`**）。不要在宿主机对本机路径单独执行 `cmake` 或 **`./scripts/menu.sh`**（及其子步骤），否则易与容器内已生成的 `build/` 缓存路径冲突。脚本说明见 **[`scripts/README.md`](scripts/README.md)**。

## 运行与构建环境（已验证）

| 项目 | 说明 |
|------|------|
| 容器 | Docker 容器 **`sy_dev`**（镜像 **`sy_env:v1`**） |
| 工程路径（容器内） | **`/workspace/ForceRes_ZY`**（与宿主机共享目录以你本机挂载为准；**编库与脚本均在此路径下执行**） |
| 操作系统 | Ubuntu 24.04（容器内） |
| CPU 架构 | **aarch64** |
| 编译器 | **GCC / G++ 13.3** |
| CMake | **3.28.x** |
| 系统依赖 | **Boost**（验证时 **1.83**）、**Threads** |

CMake **不在配置阶段联网**；`3rd/eigen` 须完整，否则 `cmake` 直接 `FATAL_ERROR`。

## 第三方库版本（随仓库）

| 库 | 版本 | 路径 |
|----|------|------|
| **Eigen** | **3.4.0** | `3rd/eigen`（`Eigen/src/Core/util/Macros.h`） |
| **Orocos KDL** | **1.5.1** | `3rd/kdl`（`config.h`） |
| **spdlog** | **1.10.0** | `3rd/spdlog`（`version.h`） |

## 目录说明

| 路径 | 含义 |
|------|------|
| `rokae_header/` | 头文件；**SDK 对外单头**为 **`rokae_header/rokae_force_controller.hpp`**（仅标准库 + Eigen，不拉其它 `rokae_header`；实现见 `rokae_src/rokae_force_controller.cpp`） |
| `rokae_src/` | 实现 |
| `cmake/` | **`kdl_sources.cmake`**：编入 `libforce_res.a` 的 KDL 源文件最小列表 |
| `scripts/` | 主入口 **[`menu.sh`](scripts/menu.sh)**（交互选 1～4）；子脚本含 `build_and_sync_example.sh`、`sync_example_from_repo.sh` 等，见 **[`scripts/README.md`](scripts/README.md)** |
| `example/` | **可单独复制**的 SDK 示例包：`CMakeLists.txt`、`demos/`；打包前在 **容器 `sy_dev` 内**仓库根运行 **[`scripts/menu.sh`](scripts/menu.sh)** 选 **1**（或按 [`scripts/README.md`](scripts/README.md) 单独调用子脚本，见 `example/README.md`） |
| `example/lib/` | 根工程生成的 **`libforce_res.a`**（`.a` 默认不提交 Git） |
| `example/demos/` | 各 demo 源与 `client_demo_macros.h` |
| `example/include/` | 对外单头（由 `scripts/sync_example_from_repo.sh` 或 **`./scripts/menu.sh` 选 2** 从 `rokae_header/` 复制）及说明，见 **`example/include/README.md`** |

## 构建主静态库（容器 `sy_dev` 内、仓库根）

以下命令在 **容器 `sy_dev`** 中、**`cd /workspace/ForceRes_ZY`**（或你的挂载等价路径）后执行。

一键编译并在成功后同步到 `example/`（**推荐**；若 `cmake` 或编译失败则不会执行同步）：

```bash
cd /workspace/ForceRes_ZY
chmod +x scripts/menu.sh   # 首次
./scripts/menu.sh          # 交互：选 1 编库并同步 example
```

或手动分步（仍在容器内）：

```bash
cd /workspace/ForceRes_ZY
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./scripts/menu.sh 2        # 仅同步 example；也可 ./scripts/sync_example_from_repo.sh
```

产物：**`example/lib/libforce_res.a`**（由根 `CMakeLists.txt` 指定 `ARCHIVE_OUTPUT_DIRECTORY`）。

### 静态库体积说明

- **Release**：单配置生成器若未指定 `CMAKE_BUILD_TYPE`，根工程会**默认 `Release`**；显式使用 `-DCMAKE_BUILD_TYPE=Release` 可得到较小 `.a`（对 `force_res` 启用 `NDEBUG`、`-g0`、`-ffunction-sections` / `-fdata-sections`）。
- **KDL**：仅编译 [`cmake/kdl_sources.cmake`](cmake/kdl_sources.cmake) 中列出的子集（Chain / 动力学 / FK / Jac / RNE 等），不再整库 `GLOB` 全部 KDL 源，以显著减小归档体积。
- **可选进一步缩小**：对发布用 `.a` 执行 `strip --strip-unneeded`（不利于后续用该 `.a` 调试）；或尝试 `-DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON`（LTO，编译更慢）。
- **测量**：[`scripts/measure_lib_size.sh`](scripts/measure_lib_size.sh) 或 **`./scripts/menu.sh` 选 3** 可打印 `libforce_res.a` 字节数与 `ls -lh`。

## 构建示例 / Demo（自包含 `example/`）

在 **容器 `sy_dev`** 内完成上节编库与同步后，在 **`example/`** 目录下构建 demo：

```bash
cd /workspace/ForceRes_ZY/example
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
# 可执行文件在 example/build/demos/ 下，例如 ./build/demos/joint_drag_test
```

将 **`example/` 整目录**（含同步后的 `3rd/`、`lib/`、`include/`）复制到其它机器后，在相同 **GCC + Boost** 环境下重复 `cd example && cmake ...` 即可。详见 **`example/README.md`**。

## 集成说明（给第三方工程）

- 链接：`libforce_res.a`，并链接 **Boost**、**pthread**（与主工程一致）。
- 编译（推荐客户路径）：`-I` 指向放置 **`rokae_force_controller.hpp`** 的目录（例如 SDK 的 `include/`），`#include "rokae_force_controller.hpp"`；并配置 **Eigen** 头路径（与编译 `.a` 时一致）。**不必**为客户配置 KDL / 其它 `rokae_header`，除非其代码直接包含内部类型。

详见 **`example/include/README.md`**。

## 许可证与声明

第三方库遵循各自许可证；业务代码版权归原权利方所有。
