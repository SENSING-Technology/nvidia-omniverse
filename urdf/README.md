# usd2urdf — Omniverse / Isaac USD 相机模型 → ROS URDF

把 NVIDIA Omniverse 提供的相机 USD 资产批量转换成 ROS 2 可直接使用的
URDF + STL + `camera_info` YAML。

## 依赖

```bash
pip install usd-core trimesh numpy scipy
```

## 用法

```bash
cd /home/nvidia/YXT/Senbot/nvidia-omniverse/urdf
python3 usd2urdf.py "../Camera Models/*.usd" -o . --mass 0.15
```

本目录下已包含全部 12 个模型的转换结果（每个模型一个同名子目录）。

参数：

| 参数 | 说明 |
|---|---|
| `patterns...` | 一个或多个 USD 文件 / glob 表达式 |
| `-o, --out` | 输出根目录（默认 `out`），每个模型一个子目录 |
| `--mass` | 生成 `<inertial>` 时使用的质量（kg，默认 0.15），惯量按包围盒立方体近似 |
| `--no-collision` | 不生成 `<collision>` |
| `--verbose` | 打印每个相机 prim 的内参 |

## 输出结构

```
out/<MODEL>/
├── <MODEL>.urdf                        # 机器人描述
├── <MODEL>_camera_info.yaml            # 单相机模型的内参（ROS camera_info 格式）
├── <MODEL>_<link>_camera_info.yaml     # 多相机模型（如 S56C 双镜头）每个相机一份
└── meshes/*.stl                        # 三角面片，单位：米
```

## 转换规则

- **坐标系**：`upAxis` 一律转换为 ROS 惯例 **Z-up**；`metersPerUnit` 烘焙进网格顶点，
  缩放后 STL 单位为米。
- **层级**：每个带几何 / 相机的 USD prim 映射为一个 `<link>` + 一个 `fixed`
  `<joint>`；变换恒等且是 `Mesh` 的 prim 会合并进父 link，保持 TF 树精简。
- **累积缩放**：USD 允许缩放挂在没有几何的祖先 `Xform` 上，而 URDF 的 joint
  无法表达 scale。转换器用 `XformCache.GetLocalToWorldTransform()` 取出每个
  prim 的**累计缩放**，烘焙进父级平移量与网格顶点，保证尺寸与 USD 世界包围盒一致。
- **相机**：每个 `Camera` prim 额外生成 `<name>_optical` 子 link，`rpy = (π, 0, 0)`，
  即 ROS 光学坐标系（Z 前 / X 右 / Y 下）。
- **内参**：读取 `omni:lensdistortion:*` 命名空间，自动识别
  `opencvPinhole` → `rational_polynomial`、`opencvFisheye` → `equidistant`。
  URDF 无法承载内参，因此写成 ROS `camera_info` YAML 旁挂文件。

## 已知损失

- 材质 / MDL 贴图不会转换，`<visual>` 只有几何体，无 `<material>` 颜色。
- `Render`、`Graph`（OmniGraph 动作图）、`Looks` 子树被跳过。
- 关节全部是 `fixed`（相机为刚体，USD 里也无 articulation）。
- `<inertial>` 是按 `--mass` 与包围盒立方体估算的近似值，非真实 CAD 惯量。
- 不处理 `instanceable` prim 的实例代理去重（几何会被展开复制）。

## 精度

以「USD 世界包围盒」对「URDF 正运动学包围盒」做校验，12 个模型的
最大偏差 < 1.3e-7 m。
