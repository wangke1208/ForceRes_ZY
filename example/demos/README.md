# 历史 Demo / 测试可执行文件

源文件位于本目录，**不由根目录 CMake 编译**；在生成 `example/lib/libforce_res.a` 后，可用本子工程单独生成各可执行文件。

```bash
cd example/demos
cmake -S . -B build -G Ninja
cmake --build build
```

产物在 `example/demos/build/` 下。
