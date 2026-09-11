# QOpenGL 机械臂可视化开发设计

## 1. 目标与范围

在现有 Qt 6.5 Widgets 空项目中构建一个桌面端机械臂三维可视化工具。第一阶段目标：

- 显示由若干连杆（Link）和转动关节（Revolute Joint）组成的机械臂；
- 用滑块、数值框或鼠标交互改变每个关节角度；
- 实时显示正确的串联运动学姿态；
- 支持鼠标旋转、平移、缩放相机，显示网格、坐标轴与末端执行器坐标系；
- 架构上允许后续替换为 STL/OBJ/glTF 网格、接入 URDF、TCP/串口数据和轨迹动画。

第一阶段不做碰撞检测、逆运动学、物理仿真或真实控制下发；它们应建立在稳定的正运动学与渲染基础之上。

## 2. 技术选型

| 层次 | 选型 | 原因 |
| --- | --- | --- |
| UI | Qt Widgets / `QMainWindow` | 与当前工程一致，适合停靠面板和工业桌面工具。 |
| OpenGL 容器 | `QOpenGLWidget` | Qt 原生集成，负责 OpenGL 上下文和重绘生命周期。 |
| OpenGL API | `QOpenGLFunctions_3_3_Core` | 固定核心 Profile，避免兼容模式 API；3.3 在桌面端兼容性好。 |
| 数学 | `QMatrix4x4`、`QVector3D`、`QQuaternion` | Qt 内置，减少第三方依赖。 |
| 着色器 | GLSL 330 Core | 用统一的 MVP、法线、材质和光照管线。 |
| 构建 | CMake + Qt 6 | 沿用已有工程。 |

建议使用 OpenGL 3.3 Core Profile；应用启动前设置格式，避免驱动默认到旧版上下文：

```cpp
QSurfaceFormat format;
format.setVersion(3, 3);
format.setProfile(QSurfaceFormat::CoreProfile);
format.setDepthBufferSize(24);
format.setSamples(4);
QSurfaceFormat::setDefaultFormat(format);
```

这段代码应在创建 `QApplication` 前执行。

## 3. 总体架构

```text
MainWindow
 ├─ RobotOpenGLWidget                 三维视图、相机、OpenGL 生命周期
 ├─ JointControlPanel                 关节滑块/SpinBox、复位、播放控制
 └─ StatusPanel                       FPS、末端位姿、告警

RobotController                       应用编排层（不含渲染 API）
 ├─ RobotModel                        连杆、关节、关节限制、当前状态
 ├─ ForwardKinematics                 根据关节角计算各坐标系世界矩阵
 └─ TrajectoryPlayer                  可选：按时间驱动关节状态

Renderer                              仅将“已计算的姿态”画出来
 ├─ ShaderProgram                     编译/绑定 GLSL
 ├─ Mesh / MeshGpuResource            CPU 顶点数据与 VAO/VBO/EBO
 ├─ RobotRenderer                     遍历 Link，提交模型矩阵和材质
 └─ GridAxisRenderer                  网格、世界/末端坐标轴
```

核心边界：`RobotModel` 和 `ForwardKinematics` 不依赖 `QOpenGLWidget` 或 OpenGL；`Renderer` 不计算关节关系，只消费每个 Link 的世界变换。这使运动学可以单元测试，也使渲染器能独立替换。

## 4. 坐标、模型和正运动学

### 4.1 统一约定

- 使用右手系：X 向右，Y 向上，Z 朝观察者或按项目约定固定；所有模型资源必须遵守同一约定。
- 角度在 UI/配置中使用度（degree），在数学计算时立即转换为弧度（radian）。
- `QMatrix4x4` 采用列向量语义，点变换写作 `worldPoint = matrix * localPoint`。
- 每个关节的局部变换定义为：`jointToParent = origin * motion(q)`。

`origin` 是零位下关节坐标系相对父连杆坐标系的固定平移/旋转；转动关节的 `motion(q)` 是围绕本地轴旋转。不要把“连杆偏移”和“关节转角”混为一个矩阵，否则后续标定和 URDF 导入会很困难。

### 4.2 数据结构

```cpp
enum class JointType { Fixed, Revolute, Prismatic };

struct Joint {
    QString name;
    JointType type = JointType::Revolute;
    int parentLink = -1;
    int childLink = -1;
    QVector3D axis {0.0f, 0.0f, 1.0f}; // 必须归一化，位于关节局部坐标系
    QMatrix4x4 origin;                 // 零位：parent link -> joint
    float valueDeg = 0.0f;
    float minDeg = -180.0f;
    float maxDeg = 180.0f;
};

struct Link {
    QString name;
    int parentJoint = -1;
    QMatrix4x4 meshOffset;             // link 坐标系 -> 网格坐标系
    std::shared_ptr<Mesh> mesh;
    Material material;
};
```

树形机械臂的递推关系如下（串联机械臂是其特例）：

```text
T_world_childLink = T_world_parentLink × T_parent_jointOrigin × R_axis(q) × T_joint_childLink
```

简化原型中可把 `T_joint_childLink` 放进 child link 的 `meshOffset`；正式模型建议显式保存，避免网格原点与连杆坐标系重合这一不可靠假设。

### 4.3 关节变化流程

```text
用户拖动 J3 滑块
  → JointControlPanel 发出 jointValueChanged(2, degree)
  → RobotController::setJointValue() 进行限位裁剪
  → ForwardKinematics::update() 重算受影响子树的 worldTransform
  → RobotOpenGLWidget::setLinkTransforms()
  → update()，下一帧 paintGL() 使用新模型矩阵绘制
```

禁止在 `paintGL()` 中修改关节状态；绘制必须是读取状态的纯过程，才能保证 UI 拖动、动画与外部数据输入不会互相干扰。

## 5. OpenGL 渲染设计

### 5.1 一帧的处理顺序

1. `initializeGL()`：加载函数表、开启深度测试和背面剔除、编译着色器、创建网格 GPU 资源。
2. `resizeGL(w, h)`：更新透视投影矩阵，`height` 至少取 1。
3. `paintGL()`：清除颜色/深度缓冲；计算 View；先绘制网格和坐标轴，再逐连杆绘制；最后绘制末端坐标轴。

每个连杆都提交：

```text
MVP = Projection × View × LinkWorld × MeshOffset
NormalMatrix = inverse(transpose(mat3(View × LinkWorld × MeshOffset)))
```

顶点着色器处理位置和法线，片元着色器采用最小的环境光 + 漫反射 + 高光（Phong/Blinn-Phong）即可。务必启用 `GL_DEPTH_TEST`；透明物体留到后续按距离排序的独立阶段。

### 5.2 网格策略

原型期不依赖外部模型：用盒子、圆柱和球体组合生成底座、连杆与关节，能够最快验证矩阵链。之后引入 `MeshLoader` 加载统一坐标约定的 STL/OBJ/glTF：

- CPU 层保存 `positions / normals / indices`；
- 第一次拥有 OpenGL 上下文时创建 VAO、VBO、EBO；
- 只在 `initializeGL()` / `cleanup()` 中创建或释放 GPU 对象；
- 不要在普通 UI 线程代码中直接调用 `gl*`，OpenGL 调用只能发生在当前 widget 上下文有效时。

## 6. UI 与交互设计

主窗口推荐如下布局：

```text
┌─────────────────────── MainWindow ───────────────────────┐
│ 工具栏：打开模型 | 零位 | 视角重置 | 播放/暂停             │
├───────────┬──────────────────────────────────────────────┤
│ 关节控制   │                                              │
│ J1 [slider]│              RobotOpenGLWidget               │
│ J2 [slider]│                                              │
│ ...        │     左键旋转 / 中键平移 / 滚轮缩放            │
│ 末端位姿   │                                              │
├───────────┴──────────────────────────────────────────────┤
│ 状态栏：FPS | 当前选中对象 | 末端 XYZ / RPY               │
└──────────────────────────────────────────────────────────┘
```

- 每个关节用 `QSlider + QDoubleSpinBox` 双向绑定，显示单位为 `°` 或 `mm`。
- 滑块范围直接来自 `Joint::minDeg/maxDeg`，控制器仍必须二次裁剪，不信任 UI 输入。
- 相机采用 orbit camera：`target + yaw + pitch + distance`；俯仰角限制在 `[-89°, 89°]`，距离限制为正数区间。
- 按住 Shift 可降低滑块步进；提供“回零”和“保存姿态”操作。

## 7. 推荐目录与类职责

```text
robot/
├─ CMakeLists.txt
├─ main.cpp
├─ ui/
│  ├─ mainwindow.h/.cpp                窗口拼装，不放运动学计算
│  └─ jointcontrolpanel.h/.cpp
├─ robot/
│  ├─ robotmodel.h/.cpp                Link/Joint 状态与模型装配
│  ├─ forwardkinematics.h/.cpp         世界矩阵计算
│  ├─ robotcontroller.h/.cpp           UI、模型、渲染之间的协调
│  └─ trajectoryplayer.h/.cpp          可选动画
├─ render/
│  ├─ robotopenglwidget.h/.cpp         QOpenGLWidget 生命周期、输入与相机
│  ├─ renderer.h/.cpp
│  ├─ mesh.h/.cpp
│  ├─ shaderprogram.h/.cpp
│  └─ camera.h/.cpp
├─ resources/
│  ├─ shaders/robot.vert
│  ├─ shaders/robot.frag
│  └─ resources.qrc
└─ tests/
   └─ tst_forwardkinematics.cpp
```

`RobotOpenGLWidget` 可以拥有 `Renderer` 和 `Camera`，但不应拥有或改写 `RobotModel`。模型经由只读快照（如 `QVector<QMatrix4x4>`）传入视图，防止渲染层成为业务状态中心。

## 8. CMake 调整

当前项目只链接了 `Core` 和 `Widgets`。实现 OpenGL 视图时，至少改为：

```cmake
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

target_link_libraries(robot PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
)
```

着色器放入 `.qrc`，通过 `:/shaders/robot.vert` 读取，避免运行目录变化导致资源加载失败。

## 9. 分阶段开发计划与验收标准

### 阶段 A：渲染最小闭环

实现 `RobotOpenGLWidget`、相机、网格/坐标轴、一个可光照立方体。

验收：窗口缩放正确、模型无闪烁、有深度遮挡，鼠标可环绕查看。

### 阶段 B：三关节原型

创建底座 + 三个圆柱连杆 + 三个 Revolute Joint，使用固定尺寸和程序化网格。

验收：改变 J1 仅绕底座轴转；改变 J2 时 J2 及其所有后续连杆一起转；角度限位生效；点击回零返回初始姿态。

### 阶段 C：工程化模型与可观测性

抽出 `RobotModel`、`ForwardKinematics`、`RobotController`，显示末端位置和局部坐标轴，为正运动学编写测试。

验收：给定一组已知关节角，末端位置与手算/标定值误差在设定容差内；渲染层无需知道关节类型。

### 阶段 D：外部模型和数据

接入模型文件（优先 glTF）、JSON/URDF 描述、轨迹回放或设备反馈。设备通信层应以信号发送“完整状态快照”，不直接操作 OpenGL。

## 10. 测试重点

- 运动学单元测试：零位、单轴 90°、两轴组合、角度裁剪、固定关节不变。
- 渲染手工测试：窗口最小化/恢复、HiDPI、缩放极限、连续快速拖动滑块。
- 数值防护：轴向量零长度、投影宽高异常、`NaN` 关节输入和损坏模型资源必须报错或回退。
- 性能基线：普通 6 轴模型在拖动关节时稳定交互；仅更新变化的矩阵，不要每帧重建 VBO 或重新编译 Shader。

## 11. 常见错误与规避

| 错误 | 后果 | 规避方式 |
| --- | --- | --- |
| 先旋转再平移的矩阵顺序错误 | 子连杆绕世界原点飞转 | 明确每个矩阵的坐标空间，按 `parent × origin × motion × child` 递推。 |
| 用世界轴旋转关节 | 上游关节转后下游轴不随动 | 关节轴定义在关节局部坐标系。 |
| 将模型顶点直接改成世界坐标 | 不能复用网格，CPU 开销大 | 顶点保持局部坐标，world transform 由 uniform 传入。 |
| 在 UI 槽函数中调用 OpenGL | 上下文可能无效、随机崩溃 | 槽函数只改模型并调用 `update()`。 |
| 将 UI 角度直接当弧度 | 姿态严重错误 | 在运动学边界统一转换，并标明单位。 |

## 12. 后续扩展接口

- **URDF 导入**：把 `<link>` 映射为 `Link`，把 `<joint origin axis limit>` 映射为 `Joint`；渲染与运动学接口无需改变。
- **逆运动学**：新增 `IKSolver::solve(targetPose, currentState)` 输出候选关节角，再经 `RobotController` 写入模型；不可让 IK 直接操作 UI 控件。
- **真实设备通信**：`RobotTransport` 负责串口/TCP、协议解析和线程；通过 Qt signal 传递角度快照，在主线程更新模型。
- **碰撞与选取**：独立 `CollisionService` 与 `Picker`，不要耦合到基本绘制循环。

## 13. 首个可运行版本的建议

优先完成“程序化三轴机械臂 + 滑块 + 正运动学 + orbit 相机”。它比直接导入复杂 CAD 网格更容易暴露坐标系和矩阵顺序错误；待其验证通过，再替换视觉资源。这样可以把“机械臂姿态是否正确”和“模型文件是否正确”两个问题分开排查。
