# Demo 目录说明

本目录源码由**工程根**的 `CMakeLists.txt` 通过 `add_subdirectory(demos)` 参与构建；生成可执行文件在 **`build/demos/`**。推荐用工程根下的 **`script/build&run.sh`**（见根目录 **`README.md`**）。

## 各文件作用

| 文件 | 作用 |
|------|------|
| `joint_drag_test.cpp` | **空载**轴空间拖动示例：模型与力控相关初始化后，使用默认（零）负载参数进行拖动与力控流程演示。 |
| `joint_drag_test_with_load.cpp` | **带载**轴空间拖动示例：在 `joint_drag_test` 基础上显式设置末端负载（质量、质心、惯量等），标定与动力学计算均按带载工况。 |
| `client_demo_macros.h` | Demo 专用日志与打印宏（基于 spdlog），避免依赖完整工程里的日志头文件；各 `*_demo.cpp` 会 `#include` 它。 |
| `CMakeLists.txt` | 声明本目录下各可执行目标、头文件路径与链接库；**实际会参与编译的 demo 以其中 `add_force_res_demo(...)` 列表为准**，且需存在同名 `.cpp`。 |

自行构建示例：

```bash
cd <工程根目录>
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
```

接口用法与业务参数**以指导书为准**。
