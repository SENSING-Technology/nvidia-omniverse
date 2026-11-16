# URDF 与 USD 简明对照

> 面向本仓库：`nvidia-omniverse/urdf/` 下由 12 个 Omniverse 相机 USD 转换出的 URDF，
> 以及与 `zed-ros2-description` 中 Stereolabs 官方 URDF 的对比。

---

## 1. URDF 是什么

**URDF = Unified Robot Description Format**，一个用 XML 写机器人描述的格式，起源并主导于 ROS 生态。

它只回答一件事：**"这个机器人的零件怎么连在一起"**。核心是两个元素：

```xml
<link name="camera_link">      <!-- 刚体：外形 + 碰撞 + 质量 -->
  <visual>   <geometry><mesh filename="..."/></geometry> </visual>
  <collision><geometry><box size="0.1 0.1 0.1"/></geometry> </collision>
  <inertial> <mass value="0.15"/> <inertia ixx="..." .../> </inertial>
</link>

<joint name="a_to_b" type="fixed">   <!-- 连接 + 自由度 -->
  <parent link="a"/>  <child link="b"/>
  <origin xyz="0 0.06 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <limit lower="-1.5" upper="1.5" effort="30" velocity="1"/>
</joint>
```

关节类型只有 6 种：`fixed` / `revolute` / `continuous` / `prismatic` / `floating` / `planar`。

**URDF 的主要表达边界**：

| 限制 | 说明 |
|---|---|
| 运动学拓扑必须是**树** | 不能直接表达闭环关节链，例如四连杆；普通差速轮机器人本身仍可用 URDF 建模 |
| **无场景 / 无世界** | 没有地面、没有光照、没有多机器人、没有环境物体 |
| **无传感器模型** | 标准 URDF 里没有 `<camera>` / `<lidar>`。相机内参只能外挂在 `camera_info` YAML 里，或塞进 Gazebo 扩展标签 |

此外它基本是静态模型描述，不执行控制或动画。控制器、硬件接口等通常由外围配置及扩展提供。URDF 材质支持纹理引用，外部 mesh 也可能携带材质，显示效果取决于资源格式与加载器；不能说它只有纯色。

---

## 2. 什么情况下用 URDF

**它的重要价值是作为 ROS 与机器人学工具链中常见的模型交换格式。**

| 场景 | 为什么用 URDF |
|---|---|
| **TF 坐标系树** `robot_state_publisher` + `tf2` | 机器人各零件的实时位姿全靠 URDF 定义的树 + 关节角推出来 |
| **RViz / Foxglove 可视化** | 显示模型、坐标轴、点云对齐 |
| **MoveIt 2 运动规划** | URDF 提供结构和碰撞几何，SRDF、运动学插件与其他配置提供规划组等信息 |
| **机械臂遥操作 / 数据采集** | 可提供结构和安装坐标关系；本身不执行遥操作或数据录制，也不代表当前项目必定采用 ROS bag |
| **`ros2_control` 硬件接口** | 关节名、限位、传动比从 URDF 读 |
| **MuJoCo / PyBullet / Gazebo / Drake / Pinocchio** | 都能吃 URDF（Gazebo 会先转 SDF） |

**什么时候不能只靠 URDF**：需要闭环机构、写实场景、传感器仿真或场景级编排时，通常需配合 USD / MJCF / SDF 或平台扩展。

---

## 3. URDF 配合什么使用

```
                ┌─────────────────────────────────────────┐
                │            URDF (+ xacro 宏)            │
                └───────────────┬─────────────────────────┘
        ┌───────────────┬───────┴───────┬───────────────┐
        ▼               ▼               ▼               ▼
  robot_state_      MoveIt 2       ros2_control     仿真器
  publisher +      (运动规划)      (硬件驱动)     (Gazebo/MuJoCo/
   tf2 + RViz                                     PyBullet/Isaac)
        │               │               │               │
        ▼               ▼               ▼               ▼
   坐标变换树      碰撞/逆解        真实电机        刚体动力学
        │
        ▼
  camera_info YAML  ←  （URDF 装不下的内参放这里）
```

典型 ROS 2 数据链路示意（不是对当前项目实际部署的确认）：

```
URDF → robot_state_publisher → /tf /tf_static
                                    │
相机图像 → camera_driver → /image_raw + /camera_info ─┐
                                                      ▼
                                    image_projection / 点云着色 / VLA 训练数据
```

---

## 4. USD 与 URDF 的关系：谁信息多？

**不是严格的父子或语法包含关系。** URDF 聚焦机器人结构，USD 的场景表达范围通常更广，但不是 URDF 的严格语法超集。

| 维度 | URDF | USD (Omniverse / Isaac Sim) |
|---|---|---|
| 本质 | XML 机器人描述，link/joint 组成树 | 分层合成（Layering / Reference / Payload / **Variant**） |
| 拓扑 | 运动学连接是树 | 场景层级是树，引用和物理约束可形成更复杂关系；仿真器支持另论 |
| 几何 | 引用外部 mesh（stl/dae） | 内嵌 mesh / 引用 / **实例化复用** |
| 材质 | 简单材质、纹理引用，依赖加载器 | `UsdShade` 材质网络；MDL 等支持依赖平台 |
| 物理 | 质量、惯量、碰撞、关节约束 | `UsdPhysics` 描述刚体和约束；PhysX 等扩展提供更多能力 |
| 传感器 | 标准 URDF 无完整传感器模型 | `UsdGeom.Camera` 有投影、焦距、孔径等；OpenCV 畸变、LiDAR、IMU 通常依赖扩展与运行时 |
| 灯光 / 渲染 | ❌ | ✅ 完整 Hydra 渲染管线 |
| 动画 / 时间轴 | ❌ | ✅ 时间采样、动画曲线 |
| 软件生态 | ROS / 机器人学 | 影视 / 数字孪生 / Omniverse / Isaac Sim |
| 可读性 | 人肉可读可改 | 需工具，二进制 usdc 不可读 |

**结论**：
- **信息量**：USD 能承载的信息种类通常更多，但某个具体文件不一定包含这些内容；URDF 本身也支持物理参数。
- **使用场景广度**：USD 跨影视、设计、仿真、数字孪生；URDF 在 ROS、运动学与规划中常用。没有统计依据时，不比较用户数量或使用频率。
- **互转**：USD → URDF 通常有损，具体保留内容取决于转换器；URDF → USD 不会自动恢复原本不存在的真实惯量、材质或传感器标定。
- **注意**：xacro 是生成 URDF 的宏工具，不是 URDF 本身。关节限位可以映射为 USD 物理属性，不能说完全没有对应表达；`package://` 资源寻址则需要工具处理。

---

## 5. ZED 的 URDF 和它的 usdz 是什么关系

**可以是同款设备面向不同工具链的描述，但具体生成关系尚未核实。** 本次只检查了本地 ZED xacro 与 STL，没有检查具体 ZED USDZ，不能断言同源 CAD、谁由谁生成，或不能转换。

| | `zed-ros2-description/urdf/*.xacro` | ZED 官方 `.usdz`（Omniverse 资产） |
|---|---|---|
| 目标用户 | ROS 2 用户 | Omniverse / Isaac Sim 用户 |
| 文件形态 | 2 个 xacro（宏库 + 入口），展开生成 URDF | USDZ 是有约束的 ZIP 资产容器，通常不压缩，可封装 USD 层和相关资源 |
| 内容重点 | TF 树语义：相机中心、左右相机、光学帧等；附加帧取决于型号与选项 | 具体 Camera、材质、贴图等内容待检查 |
| 参数化 | 一套宏支持多个型号与选项，具体以源码分支为准 | 未核实，不能假定每款独立文件或没有变体 |
| 几何 | 通过 `package://zed_description/meshes/${model}.stl` 引用 | 具体资源组织待检查 |
| 物理 | 宏源码有 **3 处 `<collision>` 定义，无 `<inertial>`**；展开数量取决于条件分支 | 是否带物理属性待检查 |
| 传感器内参 | 不在这两个 xacro 里，通常由 SDK 与 ROS 驱动提供 | 是否包含标定属性待检查 |

> 本地 `zed-ros2-description` 中未发现 USD/USDZ。这个事实只能说明仓库内没有对应资产，不能证明两种产物的历史来源。

**关键区别**：本地 ZED 描述侧重 ROS 坐标语义与型号配置；USDZ 是资产封装格式，不保证写实程度、物理完整性或标定准确性。

---

## 6. `nvidia-omniverse/urdf/` 下的 URDF 怎么来的

**全部由 `usd2urdf.py` 从 `../Camera Models/*.usd` 程序化转换生成**，不是手写的。

```bash
cd /home/nvidia/YXT/Senbot/nvidia-omniverse/urdf
python3 usd2urdf.py "../Camera Models/*.usd" -o . --mass 0.15
```

管线（基于 `pxr` / `usd-core`）：

```
USD 文件
  ├─ 读 metadata：metersPerUnit(=0.01) / upAxis(=Z)
  ├─ Pass 1：遍历 prim，用 XformCache.GetLocalToWorldTransform() 取每个 prim 的
  │           局部变换 + **累计缩放**（USD 允许缩放挂在不带几何的祖先 Xform 上，
  │           而 URDF 的 joint 无法表达 scale → 必须烘焙进顶点和父级平移）
  │           → 恒等变换的 Mesh 直接并入父 link（精简 TF 树）
  ├─ Pass 2：每个几何/相机 prim → 一个 <link> + 一个 fixed <joint>
  │           ├─ Mesh      → 三角化 → 写 meshes/<name>.stl（顶点 × 累计缩放 × mpu，单位米）
  │           ├─ Camera    → 读 omni:lensdistortion:opencvPinhole/Fisheye → camera_info YAML
  │           │              并追加 <name>_optical 子帧（rpy = π,0,0，ROS 光学坐标系）
  │           └─ 每个带几何的 link → 按 --mass + 包围盒立方体估算 <inertial> + <collision>
  └─ 输出：<MODEL>.urdf + <MODEL>[_<link>]_camera_info.yaml + meshes/*.stl
```

产物统计：**12 个 URDF / 271 个 fixed 关节 / 283 个 link（其中 145 个是纯 TF 中间帧）/ 138 个 STL / 13 份内参 YAML**。

**已完成的验证范围**：XML 可解析；历史检查比较了 USD 与 URDF 正运动学结果的整体包围盒尺寸，最大尺寸偏差约 **1.3e-7 m**。这不是逐顶点、逐 link 位姿、光学方向或实机尺度的完整验证，也未完成目标仿真器运行验收。

**重要限制**：`--mass 0.15` 当前给每个带几何的 link 都赋予 0.15 kg，并非整台相机总重。S56C 的 25 个此类 link 合计为 3.75 kg。惯量为包围盒近似，collision 复用导出网格，不是验证过的真实动力学模型。内参 YAML 来自 USD 资产，并非实机标定证明；当前还缺少常见 ROS 标定文件的 `rectification_matrix` 和 `projection_matrix`，使用前需补齐和验证。

---

## 7. 相比 ZED 的 URDF，谁信息含量多？

**分维度看，不能按标签数量判断谁更准确。下表比较的是资源组合，YAML 不属于 URDF 本体。**

| 维度 | 本仓库生成的 URDF | ZED 官方 URDF | 胜 |
|---|---|---|---|
| `<inertial>` 质量 + 惯量 | ✅ 138 个（近似值） | ❌ 0 个 | **本仓库** |
| `<collision>` 碰撞体 | 138 个，复用导出网格，待验证 | 宏中有 3 处定义，展开数量依型号而定 | 不按数量判优 |
| 相机内参 / 畸变 | ✅ 13 份 `camera_info` YAML（fx/fy/cx/cy + 4 或 8 系数 + 焦距 + FOV） | ❌ 无（SDK 运行时给） | **本仓库** |
| 单模型几何粒度 | ✅ 每模型 6–25 个独立 STL 零件 | ❌ 每型号 1 个 STL 整体 | **本仓库** |
| 型号覆盖 | 12 个 Sensing 模型（含 S56C 双目） | 多款 ZED 型号，依本地源码版本而定 | 不同产品范围 |
| 参数化 / 可复用性 | 生成物，主要通过脚本配置 | xacro 支持型号与选项配置 | ZED 更注重参数化 |
| 语义命名 | ❌ 泛化名（`Mesh`、`Mesh_1`、`Lens`、`Fakra`） | ✅ 语义名（`_camera_center`、`_left_camera_optical_frame`、`_imu_link`） | **ZED** |
| 安装/封装规范 | ❌ 相对路径 + 裸 STL | ✅ `package://zed_description/...` + ROS 包 | **ZED** |
| 光学坐标系约定 | 相对 USD Camera 采用 `rpy=(π,0,0)` | 相对相机机体帧采用 `rpy=(-π/2,0,-π/2)` | 旋转不等价，父坐标约定不同，需结合完整链判断 |
| 材质外观 | ❌ 无 | ✅ 3 个纯色 `<material>` | ZED（略） |

**一句话结论**：
- 我们多了估算惯性及资产内参旁挂数据，但不因此自动成为更可靠的物理或标定模型。
- ZED 更强调 ROS 语义、多型号参数化与包资源组织。缺少惯性对导入的影响取决于工具、静态/动态配置与固定关节合并，不能断言一定报错。

---

## 8. 这批 URDF 能用到哪些场景

| 场景 | 能否直接用 | 说明 |
|---|---|---|
| **ROS 2 TF 树 / RViz 可视化** | 需验证加载 | 可提供结构和外形；检查相对 mesh 路径解析、frame 名称和方向，全固定关节通常无需关节状态输入 |
| **相机外参标定 / 多相机对齐** | 初始几何参考 | 需实机标定和一致的图像模型、TF；资产位姿不能视为精确外参 |
| **机械臂末端挂载相机** | 需集成 | 合并描述、避免 link 重名，再按真实安装外参添加固定关节，不是简单改一行 |
| **Isaac Sim 导入做刚体/碰撞仿真** | 可尝试导入，未验收 | 检查日志、坐标、质量、碰撞；URDF 不会自动创建成像传感器，传感器配置须另做 |
| **Gazebo / MuJoCo / PyBullet** | 需适配测试 | MuJoCo 支持直接加载 URDF；各工具支持范围不同，物理参数仍须校正 |
| **MoveIt 2 碰撞场景规划** | 几何输入 | 可用于环境物体或末端工具，还需简化碰撞网格及配置碰撞关系 |
| **视觉-惯性里程计 / VIO 标定** | 不能单靠这些文件 | 还需 IMU 数据、相机与 IMU 外参、时间同步等；未验证实机双目基线 |
| **点云着色 / 深度投影验证** | 需补充验证 | 需完整标定信息、正确的原始/矫正图像模型、深度数据与 TF，不能直接保证 YAML 可用 |
| **写实渲染 / 材质展示 / 数字孪生** | ❌ | 转 URDF 时丢了 MDL 材质和灯光 → **改用原始 USD** |
| **闭环机构仿真** | ❌ | URDF 拓扑限制，且相机本来就是刚体树 |
| **VLA 模型训练（仿真侧）** | ⚠️ | URDF 只提供场景骨架，渲染与传感器仍需 USD / Isaac 侧完成 |

**建议**：在 Omniverse / Isaac Sim 中优先检查原始 `../Camera Models/*.usd`，其保留了本次转换丢弃的材质与 Camera 属性；仍须确认平台版本、资源依赖和传感器配置，不能保证打开即可正确成像。这批 URDF 的价值是作为接入 ROS 与机器人学工具链的起点，而不是替代原始 USD 或实机标定。
