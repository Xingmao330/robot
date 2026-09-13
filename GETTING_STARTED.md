# 项目上手指南

这份指南面向刚接手代码的开发者，目标是在半小时内理解项目如何从 `irb4600.glb` 加载模型、渲染机械臂、响应关节变化，并通过 Gizmo 驱动 IK 与轨迹播放。

日常运行和交互操作请看 [README](README.md)。

## 先建立整体认知

项目是一个 Qt Widgets 桌面程序，中央区域是 `QOpenGLWidget`，左侧停靠栏负责关节和轨迹控制。

```text
MainWindow（界面编排、控制流）
  ├─ RobotModel（关节状态、GLB 节点世界变换、运动学状态）
  ├─ RobotOpenGLWidget（OpenGL 渲染、鼠标交互、拾取、Gizmo）
  │    ├─ OrbitCamera（视角）
  │    ├─ ScenePicker（组件拾取）
  │    └─ TargetGizmo（末端位置目标）
  ├─ IKSolver（目标位置 → 六轴角度）
  └─ TrajectoryPlayer（记录点 → 循环插值角度）
```

最重要的约定是：**模型姿态的唯一来源是 `RobotModel` 的 J1–J6 角度**。任何输入（滑块、IK、轨迹播放器）最终都应写入它，然后调用 `MainWindow::updateRobotView()` 刷新 OpenGL 视图。

## 从程序入口开始读

1. [main.cpp](main.cpp) 创建 `QApplication` 与 `MainWindow`。
2. [mainwindow.cpp](mainwindow.cpp) 创建 `RobotOpenGLWidget`，从可执行目录或当前目录加载 `irb4600.glb`。
3. `RobotModel::loadGlbFile()` 调用 `TinyGltfLoader` 读取模型，并建立 J1–J6 的局部旋转轴。
4. `MainWindow::updateRobotView()` 获取 `RobotModel::glbNodeTransforms()`，交给 `RobotOpenGLWidget::setGlbNodeTransforms()`。
5. `RobotOpenGLWidget::paintGL()` 遍历节点与网格 primitive，结合节点变换、材质参数和光照绘制。

可以先在 `MainWindow::updateRobotView()`、`RobotModel::glbNodeTransforms()` 和 `RobotOpenGLWidget::paintGL()` 下断点，观察一次滑块变化完整经过的调用链。

## 模型数据与渲染

### GLB 加载

[robot/tinygltfloader.cpp](robot/tinygltfloader.cpp) 是 tinygltf v3 的项目适配层：将 glTF 的 node、mesh primitive、顶点、索引和 PBR 材质转换为项目的 `GlbModel` 数据结构。

一个 glTF 节点可能包含多个 primitive，项目用 `GlbNode::meshIndices` 保存它们。渲染时逐 primitive 读取 `GlbMesh::baseColor`、`metallic`、`roughness`，这是 ABB 字样与机身可显示不同颜色的原因。

### 节点、关节与缩放

[robot/robotmodel.cpp](robot/robotmodel.cpp) 中，名字为 `link1` 到 `link6` 的节点分别叠加 J1–J6 的局部旋转。节点世界变换按：

```text
worldTransform = parentWorldTransform × localTransform × jointRotation
```

计算。模型导出单位为毫米，因此最终渲染与运动学计算会统一缩放为米制视图单位。替换模型时优先保持 `link1`…`link6` 命名；否则需同步修改该映射逻辑。

## 控制与交互数据流

### 滑块

`QSlider::valueChanged` → `RobotModel::setJointAngle()` → `updateRobotView()`。

这是最短、最适合验证节点层级与关节轴向的路径。

### 选择与末端位置 Gizmo

鼠标点击模型后，`ScenePicker::pickNode()` 使用射线与节点 AABB 相交选择组件。`TargetGizmo` 显示在所选节点处：

- 拖动红/绿/蓝轴：沿当前 Gizmo 局部轴移动目标；
- 拖动三轴交点的白点：在相机观察平面内自由移动目标。

目标位置变化通过 `RobotOpenGLWidget::ikTargetMoved` 传给 `MainWindow`；后者调用 `IKSolver::solvePosition()`，将解出的关节角写回 `RobotModel`。

### 轨迹

[robot/trajectoryplayer.cpp](robot/trajectoryplayer.cpp) 只记录关节角数组，不记录笛卡尔坐标。每段采用五次缓入缓出曲线：

```text
ease(t) = 6t⁵ - 15t⁴ + 10t³
```

最后一点平滑回到第一点后继续循环。播放器发出 `jointAnglesChanged`，`MainWindow` 更新模型和滑块；滑块拖动、Gizmo 拖动或新的 IK 请求会停止播放，避免多来源同时控制关节。

## 代码导航表

| 需要修改的内容 | 首先查看 |
| --- | --- |
| 界面布局、按钮与信号连接 | [mainwindow.cpp](mainwindow.cpp) |
| GLB 解析、材质或贴图 | [robot/tinygltfloader.cpp](robot/tinygltfloader.cpp)、[robot/glbmodel.h](robot/glbmodel.h) |
| 关节轴、节点映射、坐标单位 | [robot/robotmodel.cpp](robot/robotmodel.cpp) |
| IK 精度、阻尼、限位 | [robot/iksolver.cpp](robot/iksolver.cpp) |
| Gizmo 外观与鼠标拖动规则 | [render/targetgizmo.cpp](render/targetgizmo.cpp) |
| 拾取、OpenGL 绘制、着色器 | [render/robotopenglwidget.cpp](render/robotopenglwidget.cpp) |
| 相机行为 | [render/orbitcamera.cpp](render/orbitcamera.cpp) |
| 记录点速度、暂停和循环 | [robot/trajectoryplayer.cpp](robot/trajectoryplayer.cpp) |

## 建议的开发顺序

1. 确认 GLB 中 `link1` 到 `link6` 的层级与每个局部轴方向。
2. 用滑块逐轴验证旋转是否正确，特别是 J4、J5、J6。
3. 验证选择 `link6` 后，Gizmo 是否随关节姿态更新。
4. 使用单轴 Gizmo 拖动测试 IK 的误差与奇异姿态表现。
5. 再调整 `IKSolver` 的 `damping`、`maximumIterations`、`positionTolerance`。
6. 最后调节 `TrajectoryPlayer` 的单段时长和速度规划。

## 常见问题

**模型全部黑色或颜色不对**：检查 primitive 是否读取了材质，确认每个 primitive 均单独绘制；着色器需要同时使用法线、金属度和粗糙度。

**末端拖动后 Gizmo 不跟随**：检查 `RobotOpenGLWidget::setGlbNodeTransforms()` 是否在每次姿态更新后刷新 Gizmo 的位置和局部轴向。

**关节像摆动而不是绕轴转**：检查 `RobotModel::loadGlbFile()` 中该关节的局部轴定义，以及 GLB 节点的父子层级。

**IK 不收敛**：先确认目标在机械臂工作空间内，再适度增大阻尼或迭代次数；接近奇异位姿时优先采用较小的 Gizmo 拖动增量。
