# UI 参数说明

本文档解释 BioMotion 各个界面里出现的所有数值徽章/字段的含义、单位、来源和典型量级。涵盖 **实时跟踪界面 (`ContentView`)**、**Offline 分析界面 (`OfflineAnalysisView`)** 和 **Session 图表界面 (`SessionChartsView`)**。

> 参数定义的源头大多在 `BioMotion/Nimble/NimbleEngine.swift` 顶部的 `@Published` 字段上，对应注释里写明了物理单位与合理范围。

---

## 整体流水线（理解参数的前提）

```
ARKit 91 关节 → 1-euro 滤波 → Nimble IK（求解 DOF 角度）
            → Nimble ID（求解关节力矩 + GRF）
            → 静态优化 SO（OSQP 求解肌肉激活）
```

| 缩写 | 全称 | 含义 |
|------|------|------|
| **DOF** | Degree of Freedom | 关节自由度，例如 `hip_flexion_r`、`knee_angle_l` |
| **IK**  | Inverse Kinematics | 由 marker 位置反解 DOF 角度 |
| **ID**  | Inverse Dynamics  | 由 q / dq / ddq 反解关节力矩 |
| **SO**  | Static Optimization | 把关节力矩分配到 80 根肌肉，求解每根肌肉的激活度 |
| **GRF** | Ground Reaction Force | 地面反作用力（脚底受力） |
| **CoP** | Center of Pressure | 压力中心 |

---

## 1. 实时跟踪界面 (`ContentView.trackingView`)

### 1.1 顶部状态栏

| 徽章 | 含义 | 单位 | 典型值 |
|------|------|------|--------|
| 跟踪状态 (左上) | ARKit body tracking 是否就绪。绿点=已跟踪，橙点=未跟踪 | — | — |
| `IK xx.xms` | 最近一次 IK 求解耗时（包含 SG 滤波 + 数值微分） | 毫秒 (ms) | ~1 ms |
| `tracked / total` | 当前帧中被 ARKit 视为「已跟踪」的关节数 / 总关节数 | 个 | 多数为 91/91，部分为 70+/91 |

### 1.2 精度诊断徽章 (`AccuracyBadge`，两行)

绿色表示在合理范围内，橙色表示偏离健康区间。阈值与目标量级在 `ContentView.swift` 行 79–86 注释里写得最清楚，照搬如下：

#### 第 1 行

| 字段 | 含义 | 单位 | 健康阈值 | 备注 |
|------|------|------|----------|------|
| `residual` | IK marker 残差的 RMS（虚拟 marker 与 ARKit 关节位置的均方根距离） | 米 (m) | `< 0.05 m` 为绿，目标 `0.01–0.03 m`（ARKit 自身精度极限） | 反映「Nimble 解出的姿态与 ARKit 观测有多吻合」。来源：`NimbleEngine.ikMarkerResidualMeters` |
| `max \|τ\|/m` | 当前帧所有关节中 **最大绝对力矩**，按总质量归一化 | Nm/kg | `< 5 Nm/kg` 为绿，生理走路/下蹲应在 `1–3 Nm/kg` | `> 10` 通常意味着 GRF 缺失或 ddq 噪声爆炸 |
| `mass` | 加载并缩放后骨架的总质量 | 千克 (kg) | 模型自带，不会跳变 | Rajagopal2016 默认 ~75 kg，会按身高缩放 |

#### 第 2 行

| 字段 | 含义 | 单位 | 健康阈值 | 备注 |
|------|------|------|----------|------|
| `L/R load` | 左脚 \| 右脚 垂直 GRF 占体重的比例（显示为 `0.00\|0.00` 形式） | 体重的倍数 (无量纲) | `\|L+R-1.0\| < 0.3` 或 `L+R < 0.1`（飞行相）为绿 | 双足站立 `≈0.5\|0.5`；单脚承重 `≈1.0\|0.0`；起跳冲击可达 `2–3` |
| `root res` | ID 求解后浮动根关节 6D 残差，按质量归一 | Nm/kg | `< 0.5 Nm/kg` 为绿 | 衡量 GRF 与运动学的一致性。残差大 → GRF 估计错或 ddq 噪声大 |

### 1.3 肌肉激活条 (`MuscleActivationBar`)

显示 12 块关键下肢肌肉（左右共 6 对：soleus、gastroc med、tibialis ant、vastus med、rectus fem、glut max）。

| 元素 | 含义 | 单位 |
|------|------|------|
| 柱体高度 | 该肌肉的 **激活度 a** | 无量纲，`0.0 – 1.0`（即 0%–100%） |
| 颜色 | `<30%` 蓝、`<60%` 绿、`<80%` 黄、`≥80%` 红 | — |
| 百分比文本 | `a × 100%` | % |

> 激活度是 OSQP 静态优化的结果：在「关节力矩 = Σ(肌力 × 力臂)」约束下，最小化 Σaᵢ²。生理上 `a ≈ 0.05–0.4` 的多块肌肉协同最常见。

### 1.4 IK / ID 数据面板 (`IKReadoutPanel`)

| 字段 | 含义 | 单位 |
|------|------|------|
| 标题右侧 `err: x.x mm` | 当前帧 IK marker RMS 误差（与 `residual` 同源，仅单位换算） | 毫米 (mm) |
| 每个 DOF 上排：`xxx.x°` | IK 解出的关节角度 | 度 (°)（内部存弧度，显示前 ×180/π） |
| 每个 DOF 下排（黄色）：`x.x Nm` | ID 解出的关节力矩 | 牛米 (Nm) |

显示的 7 个关键 DOF：`hip_flexion_{r,l}`、`knee_angle_{r,l}`、`ankle_angle_{r,l}`、`lumbar_extension`。

### 1.5 关节位置面板 (`JointReadoutPanel`，关闭 IK 面板时显示)

| 字段 | 含义 | 单位 |
|------|------|------|
| 关节名 | ARKit 91 关节集合中的名称（仅显示 `isTracked == true`） | — |
| `x, y, z` | ARKit 世界坐标系下的 3D 位置 | 米 (m) |

ARKit 世界坐标：原点在 session 启动位置，Y 轴朝上，单位米。

### 1.6 录制条

| 元素 | 含义 |
|------|------|
| 红色圆点 + `m:ss.f` | 录制时长 |
| `Nf` | 已录制帧数 |
| `Export` | 导出 `.trc` (marker) + `.mot` (IK 角度) + `.sto` (ID 力矩) 三个 OpenSim 文件 |

---

## 2. Offline 分析界面 (`OfflineAnalysisView`)

### 2.1 OpenCap Import / Selected / Keys

| 字段 | 含义 |
|------|------|
| `Model` | 选中的 `.osim` 模型文件名 |
| `Motion` | `.mot` 关节角度时间序列文件名 |
| `Video` | `.mov` / `.mp4` 视频文件名 |
| `Keys` | `.keypoints.json` + 帧数 + 抓取帧率（Hz）。来源：OpenPose-25 风格的 2D 关键点 |
| `Selected` | 上次导入时用户选择的文件夹名 |

### 2.2 Prepared Motion

| 字段 | 含义 | 单位 |
|------|------|------|
| `Frames` | `.mot` 文件包含的帧数 | 帧 |
| `Duration` | 末帧时间戳 - 首帧时间戳 | 秒 (s) |
| `FPS` | 估算帧率 = Frames / Duration | Hz |

### 2.3 Playback

| 字段 | 含义 |
|------|------|
| 视频上方覆盖层 | 用 `.keypoints.bin` 里的 2D 像素坐标画的骨架 |
| `Frame N / M` | 当前帧索引 / 总帧数 |
| `x.xxxs` | 当前帧的播放时间戳 |

### 2.4 Batch Analysis

| 字段 | 含义 | 单位 | 备注 |
|------|------|------|------|
| `xxx frames` (右上) | 完成批处理的帧数 | 帧 | — |
| `Max \|τ\|/m` | **整段** 中所有帧、所有关节的力矩绝对值最大值 / 质量 | Nm/kg | 与实时界面同义但取整段最大值 |
| `Root Res` | 整段中根残差最大值 / 质量 | Nm/kg | < 0.5 健康 |
| `Mass` | 模型总质量 | kg | — |
| `SO Converged` | OSQP 静态优化收敛的帧数 / 总帧数 | 帧 | 比例越高越好；不收敛通常因力臂矩阵奇异或力矩超出肌肉总能力 |
| `Export` | 导出 CSV 到 App 沙盒（路径在 `Export` 行显示） | — | — |

### 2.5 Live Frame（**这就是你问的那块**）

> Offline 模式下 IK 被绕开（直接读 `.mot` 已有的关节角度），所以这里 **不显示 IK 耗时和 IK 残差**——见 `OfflineAnalysisView.swift` 行 264–267 的注释。

| 徽章 | 含义 | 单位 | 典型值 |
|------|------|------|--------|
| `ID` | 当前播放帧的 **Inverse Dynamics** 求解耗时 | 毫秒 (ms) | ~0.1 ms |
| `SO` | 当前播放帧的 **Static Optimization**（OSQP 肌肉求解）耗时 | 毫秒 (ms) | ~0.5 ms |
| `Max \|τ\|/m` | 当前帧 max\|关节力矩\| / 总质量 | Nm/kg | 1–3 健康，>10 异常 |
| `Root Res` | 当前帧根关节 6D 残差 / 总质量 | Nm/kg | < 0.5 健康 |

下方 `MuscleActivationBar`、`IKReadoutPanel` 与实时界面完全相同（角度 °、力矩 Nm、激活度 0–100%）。

---

## 3. Session 图表界面 (`SessionChartsView`)

### 3.1 Session Summary

| 字段 | 含义 | 单位 |
|------|------|------|
| `Duration` | 录制总时长 | 秒 (s) |
| `Frames` | 录制总帧数 | 帧 |
| `FPS` | 平均帧率 = Frames / Duration | Hz |
| `Avg IK Err` | 整段所有帧 IK marker 残差的算术平均（×1000 转 mm） | 毫米 (mm) |

### 3.2 图表

| 图 | X 轴 | Y 轴 | 单位 |
|----|------|------|------|
| Joint Angle | Time | 选中 DOF 的 IK 角度（弧度 × 180/π） | 秒 vs 度 (°) |
| Joint Torque | Time | 选中 DOF 的 ID 力矩 | 秒 vs 牛米 (Nm) |
| IK Marker Error | Time | 当帧 RMS marker 残差 ×1000 | 秒 vs 毫米 (mm) |

DOF 选择器中可用的 DOF 取决于模型，常见为：髋屈/伸 (`hip_flexion`)、髋外展 (`hip_adduction`)、髋内外旋 (`hip_rotation`)、膝 (`knee_angle`)、踝 (`ankle_angle`)、腰椎伸展 (`lumbar_extension`)，每个左右两侧 (`_r` / `_l`)。

---

## 4. 单位 / 符号速查

| 符号 | 全称 | 物理量 | 单位 |
|------|------|--------|------|
| `q` | generalized coordinates | DOF 位置（角度或位移） | rad / m |
| `dq` | generalized velocity | DOF 速度 | rad/s / m/s |
| `ddq` | generalized acceleration | DOF 加速度 | rad/s² / m/s² |
| `τ` | joint torque | 关节力矩 | Nm |
| `F` | force | 力 | N |
| `a` | muscle activation | 肌肉激活度 | 无量纲 (0–1) |
| `residual` | marker residual | IK 拟合残差 | m (实时显示) / mm (面板显示) |

---

## 5. 健康范围速查（绿/橙阈值）

| 指标 | 绿（健康） | 橙（异常） | 含义 |
|------|------------|------------|------|
| `residual` (m) | `< 0.05` | `≥ 0.05` | IK 拟合差，可能 marker 标定漂移 |
| `max \|τ\|/m` (Nm/kg) | `< 5` | `≥ 5` | ID 输出在生理范围内 |
| `root res` (Nm/kg) | `< 0.5` | `≥ 0.5` | GRF 与运动学一致 |
| `L/R load` (体重倍数) | `\|L+R-1\| < 0.3` 或 `L+R < 0.1` | 其他 | 站立期承重接近 1.0；飞行期接近 0 |
| `SO Converged` | `100%` | `< 100%` | OSQP 全部帧成功收敛 |

> 阈值在 `ContentView.swift` 的 `AccuracyBadge` 调用处硬编码，调整阈值改那里即可。
