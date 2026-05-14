# 力控示例（轴空间拖动）

本示例支持**轴空间拖动**。当前提供两个 demo，用于验证轴空间拖动：

| Demo | 说明 |
|------|------|
| `joint_drag_test` | 空载 |
| `joint_drag_test_with_load` | 带载 |

参数含义、操作流程与安全注意等**以指导书为准**。

## 脚本怎么跑

在解压后的本包目录中，用 **bash** 执行 `script/build&run.sh`（文件名含 `&`，请加引号，避免被当成后台符）：

```bash
bash "./script/build&run.sh"          # 交互选 1 或 2
bash "./script/build&run.sh" 1        # 直接跑空载 demo
bash "./script/build&run.sh" 2        # 直接跑带载 demo
```

若已 `chmod +x`，也可：`'./script/build&run.sh'`。

脚本用「`script/` 的上一级」定位工程根，在该目录下删除 `build/`、重新配置并编译，然后运行对应可执行文件。依赖工具链与库版本**以指导书为准**。
