# 对外头文件说明

## 用户源码里通常只写一行

```cpp
#include "rokae_header/rokae_force_controller.hpp"
```

这里的「**一个头文件**」指的是：**你只需主动 `#include` 这一份对外 API 入口**；编译器仍会根据该文件里的 `#include` 去打开 `rokae_header/` 下的其它头文件（例如 `basic_interface.hpp`、`data_structure_define.hpp` 等）。这是 C++ 的常见行为，**与是否已经链接 `libforce_res.a` 无关**——静态库里是目标文件，不会替代这些声明与类型定义。

因此，在集成本库时，除 `example/lib/libforce_res.a` 外，仍需在编译选项中加入 **仓库根目录**（或你安装前缀下的等价布局），使上述 `#include "rokae_header/..."` 能解析，例如：

```text
-I/path/to/ForceRes_ZY
```

并保留对 **Eigen / spdlog / KDL 头文件路径** 的配置（与主工程 `CMakeLists.txt` 中一致）。

若将来要做到「磁盘上只交付一个物理 `.hpp` 文件」，需要对接口做专门改造（例如 C API、PIMPL、或合并/裁剪头文件），属于 API 设计层面的工作，而不是仅靠链接静态库即可。
