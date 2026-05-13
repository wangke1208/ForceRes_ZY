# 对外头文件说明

## 推荐给 SDK 客户：瘦对外头 + 静态库

交付第三方时，优先只提供：

- **`rokae_force_controller_public.hpp`**（内容与本仓库 `rokae_header/rokae_force_controller_public.hpp` 相同）
- **`libforce_res.a`**

用户源码中（`-I` 指向放置该头的目录，例如 SDK 的 `include/`）：

```cpp
#include "rokae_force_controller_public.hpp"
```

该头**不** `#include` 本仓库其它 `rokae_header`（不拉 `basic_interface`、`data_structure_define`、KDL 等）；仅依赖 **C++ 标准库**与 **Eigen**（声明中使用了 `Eigen::MatrixXd` 等）。

因此客户编译选项仍需：

- **`-I` 到能解析上述 `#include` 的目录**
- **Eigen 头文件路径**（与编译 `libforce_res.a` 时一致的 Eigen 3.x）
- 链接 **`libforce_res.a`**，以及 **Boost**、**pthread** 等与主工程一致项

**不是**「磁盘上零依赖」：接口里仍带 Eigen 类型，客户必须能编译到 Eigen。

若将来希望客户**连 Eigen 头都不需要**，需再改对外声明（例如改为 `double*` / opaque 句柄），属于接口演进。

---

## 仓库内「客户环境」模拟（整个 `example/` 目录）

1. 在仓库根构建主库，得到 **`example/lib/libforce_res.a`**。  
2. 在仓库根执行 **`./example/sync_from_repo.sh`**：将瘦头、`3rd/eigen`、`3rd/spdlog` 拷入 `example/`（`.a` 已在 `example/lib` 时跳过复制）。  
3. **`cd example && cmake -S . -B build && cmake --build build`**：不依赖上级 `rokae_src` / `rokae_header` 路径。  

将填好的 **`example/`** 整夹复制到其它路径即可离线构建 demo。Demo 仅 `#include "rokae_force_controller_public.hpp"` + **`client_demo_macros.h`**（本仓库打印用）。详见 **`example/README.md`**。

---

## 仓库内聚合入口：`rokae_force_controller.hpp`

本仓库内部与历史 demo 仍可使用：

```cpp
#include "rokae_header/rokae_force_controller.hpp"
```

该文件在瘦头之上**额外** `#include` `basic_interface.hpp`、`data_structure_define.hpp` 等，便于蹭到日志宏、内部类型等；**不建议**作为给客户的唯一交付头。
