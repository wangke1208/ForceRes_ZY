# ForceRes_ZY

珞石力控相关 C++ 算法库（`rokae_header` / `rokae_src`）及第三方依赖源码。根目录 CMake **仅**构建静态库 `libforce_res.a`（**不包含** `example/` 下任何源码）。示例与历史 demo 在 **`example/demos/`**，需单独用子目录 CMake 构建。

## 运行与构建环境（已验证）

| 项目 | 说明 |
|------|------|
| 容器 | Docker 容器 **`sy_dev`**（镜像 **`sy_env:v1`**） |
| 工程路径（容器内） | **`/workspace/ForceRes_ZY`**（与宿主机共享目录以你本机挂载为准） |
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
| `rokae_header/` | 头文件；对外入口为 **`rokae_header/rokae_force_controller.hpp`**（见 `example/include/README.md` 说明「一行 include」与依赖链） |
| `rokae_src/` | 实现 |
| `3rd/` | Eigen、KDL、spdlog |
| `example/lib/` | 根工程生成的 **`libforce_res.a`** 输出目录（`.a` 默认不提交 Git） |
| `example/demos/` | 各 demo / 测试源文件及**独立** `CMakeLists.txt` |
| `example/include/` | 仅说明文档，**不**再复制整棵头文件树 |

## 构建主静态库（仓库根）

```bash
cd /path/to/ForceRes_ZY
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
```

产物：**`example/lib/libforce_res.a`**（由根 `CMakeLists.txt` 指定 `ARCHIVE_OUTPUT_DIRECTORY`）。

## 构建示例 / Demo（`example/demos`）

须**先**完成上一步以生成 `example/lib/libforce_res.a`。

```bash
cd example/demos
cmake -S . -B build -G Ninja
cmake --build build
# 可执行文件在 example/demos/build/ 下，例如 ./build/joint_drag_test
```

各 demo 源文件仍是**彼此独立的** `main`，与之前根目录单文件测试用法一致。

## 集成说明（给第三方工程）

- 链接：`libforce_res.a`，并链接 **Boost**、**pthread**（与主工程一致）。
- 编译：`-I` 到仓库根（或安装前缀），使 `#include "rokae_header/rokae_force_controller.hpp"` 成立；并配置 **Eigen / spdlog / KDL** 头文件路径（同主工程）。

详见 **`example/include/README.md`**。

## 许可证与声明

第三方库遵循各自许可证；业务代码版权归原权利方所有。
