# 对外头文件说明

## 推荐给 SDK 客户：单对外头 + 静态库

交付第三方时，优先只提供：

- **`rokae_force_controller.hpp`**（与本仓库 `rokae_header/rokae_force_controller.hpp` 相同）
- **`libforce_res.a`**

用户源码中（`-I` 指向放置该头的目录，例如 SDK 的 `include/`）：

```cpp
#include "rokae_force_controller.hpp"
```

该头**不** `#include` 本仓库其它 `rokae_header`（不拉 `basic_interface`、`data_structure_define`、KDL 等）；仅依赖 **C++ 标准库**与 **Eigen**（声明中使用了 `Eigen::MatrixXd` 等）。库实现 `rokae_force_controller.cpp` 在仓库内单独包含内部头。

因此客户编译选项仍需：

- **`-I` 到能解析上述 `#include` 的目录**
- **Eigen 头文件路径**（与编译 `libforce_res.a` 时一致的 Eigen 3.x）
- 链接 **`libforce_res.a`**，以及 **Boost**、**pthread** 等与主工程一致项

**不是**「磁盘上零依赖」：接口里仍带 Eigen 类型，客户必须能编译到 Eigen。

若将来希望客户**连 Eigen 头都不需要**，需再改对外声明（例如改为 `double*` / opaque 句柄），属于接口演进。

---

## 仓库内「客户环境」模拟（整个 `example/` 目录）

以下均在 **Docker 容器 `sy_dev`** 内、仓库根 **`/workspace/ForceRes_ZY`**（或等价挂载路径）执行。

1. 在仓库根构建主库，得到 **`example/lib/libforce_res.a`**（推荐 **`./scripts/menu.sh` 选 1**）。  
2. 若仅更新头与 `3rd/`、不重新编库，可 **`./scripts/menu.sh` 选 2** 或 **`./scripts/sync_example_from_repo.sh`**。  
3. **`cd example && cmake -S . -B build && cmake --build build`**：不依赖上级 `rokae_src` / `rokae_header` 路径。  

将填好的 **`example/`** 整夹复制到其它路径即可离线构建 demo。Demo 仅 `#include "rokae_force_controller.hpp"` + **`client_demo_macros.h`**（本仓库打印用）。详见 **`example/README.md`**；脚本总览见 **`scripts/README.md`**。
