# Demo 可执行文件

源文件在本目录；**不由根目录 CMake 编译**，而在 **`example/` 根** 的 `CMakeLists.txt` 中通过 `add_subdirectory(demos)` 统一配置。

在已生成 `example/lib/libforce_res.a` 并完成同步（`./scripts/menu.sh` 选 1 或 2）后，在容器内执行：

```bash
cd /workspace/ForceRes_ZY/example
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
```

可执行文件在 **`example/build/demos/`** 下（例如 `joint_drag_test`、`joint_drag_test_with_load`、`joint_impedence_demo`、`robot_cfg_model_init_smoke`）。

具体目标列表以本目录 **`CMakeLists.txt`** 为准。
