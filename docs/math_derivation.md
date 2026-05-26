# 柔性拍摄多点位纠偏 — 数学推导

## 1. 坐标系定义

| 符号 | 含义 | 单位 |
|---|---|---|
| $\{B\}$ | 机械臂基座坐标系 (Base)，随龙门架平移 | mm |
| $\{F\}$ | 机械臂法兰末端坐标系 (Flange) | mm |
| $\{C\}$ | 相机坐标系 (Cam) | mm |
| $\{T\}$ | AprilTag 目标坐标系 (Tag) | mm |

龙门架位置记作 $G = (G_x, G_y, G_z)$，单位为 mm。龙门架仅做平移，无旋转。

**核心约定：**

- 机械臂 `GetPosition()` 返回的位姿始终相对于**当前基座** $\{B_{cur}\}$
- 欧拉角顺序为 $XYZ$（外旋 $ZYX$，即 $R = R_z \cdot R_y \cdot R_x$）
- 角度单位为**度** (deg)，位置单位为**毫米** (mm)
- Tag 检测 `tvec` 为**米**，`rvec` 为**弧度** (OpenCV 标准)

---

## 2. 已知量

| 符号 | 来源 | 说明 |
|---|---|---|
| $P = (x, y, z, rx, ry, rz)$ | `robot.GetPosition()` | 机械臂当前位姿 (mm, deg)，相对于当前基座 |
| $\mathbf{T}_F^C$ | 手眼标定 (离线) | 法兰 → 相机的 4×4 齐次矩阵，常量 |
| $(\mathbf{t}_{cam}^{tag}, \mathbf{r}_{cam}^{tag})$ | Tag 检测 | 相机系下的平移 (m) 和旋转向量 (rad) |
| $G = (G_x, G_y, G_z)$ | PLC / 龙门架控制器 | 当前龙门架坐标 (mm) |

---

## 3. 基础变换公式

### 3.1 位姿 → 齐次矩阵

给定位姿 $P = (x, y, z, r_x, r_y, r_z)$ (mm, deg)，对应的齐次矩阵为：

$$\mathbf{T}_B^F(P) = \begin{bmatrix} \mathbf{R} & \mathbf{t} \\ \mathbf{0} & 1 \end{bmatrix}$$

其中：

$$\mathbf{R} = R_z(r_z) \cdot R_y(r_y) \cdot R_x(r_x)$$

$$R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{bmatrix} \quad R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix} \quad R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

$$\mathbf{t} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}$$

> 角度在计算前由度转换为弧度。

### 3.2 Tag 检测 → 齐次矩阵

$$\mathbf{T}_C^T = \begin{bmatrix} \mathbf{R}(\mathbf{r}_{cam}^{tag}) & \mathbf{t}_{cam}^{tag} \cdot 1000 \\ \mathbf{0} & 1 \end{bmatrix}$$

其中 $\mathbf{R}(\mathbf{r})$ 为 Rodrigues 旋转向量→旋转矩阵：

$$\theta = \|\mathbf{r}\|, \quad \mathbf{k} = \frac{\mathbf{r}}{\theta}$$

$$\mathbf{R} = \mathbf{I} + \sin\theta \cdot [\mathbf{k}]_\times + (1 - \cos\theta) \cdot [\mathbf{k}]_\times^2$$

### 3.3 坐标变换链

Tag 在基座坐标系中的位姿：

$$\boxed{\mathbf{T}_B^T = \mathbf{T}_B^F \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^T}$$

---

## 4. 龙门架基座偏移

龙门架移动 $\Delta G = G_j - G_i$ 时，基座坐标系随之平移 $\Delta G$。

点 $p$ 在世界中固定，其在两个基座系下的坐标关系为：

$$\mathbf{p}^{B_j} = \mathbf{p}^{B_i} + (G_i - G_j)$$

等价地，基座系 $\{B_i\}$ 到 $\{B_j\}$ 的齐次变换为：

$$\boxed{\mathbf{T}_{B_j}^{B_i} = \begin{bmatrix} \mathbf{I} & G_i - G_j \\ \mathbf{0} & 1 \end{bmatrix}}$$

> 推导：$\mathbf{p}^{B_j} = \mathbf{p}^{B_i} + (G_i - G_j) = \mathbf{I} \cdot \mathbf{p}^{B_i} + (G_i - G_j)$，写成齐次形式即得上式。

---

## 5. 示教阶段

### 5.1 记录标准点 — `recordStandardPoint`

**输入：** $P_{std}$、$(\mathbf{t}_{std}, \mathbf{r}_{std})$、$G_{std}$

机械臂在标准龙门架位置，位姿 $P_{std}$ 相对于 $\{B_{std}\}$。

直接应用变换链（式 3.3）：

$$\boxed{\mathbf{T}_{B_{std}}^{T_{std}} = \mathbf{T}_B^F(P_{std}) \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^T(\mathbf{t}_{std}, \mathbf{r}_{std})}$$

保存 $G_{std}$ 和 $\mathbf{T}_{B_{std}}^{T_{std}}$（即配方中的 `tBaseTagStd`）。

### 5.2 添加拍照点 — `addPhotoPoint`

**输入：** $P_i$、$G_i$

存储原始数据（位姿和龙门架位置），不做计算。

### 5.3 完成示教，计算相对变换 — `finishTeaching`

对每个拍照点 $i$，分两步计算。

**第一步：将拍照点法兰位姿从 $\{B_i\}$ 转换到 $\{B_{std}\}$**

$$\Delta G_i = G_i - G_{std}$$

$$\mathbf{T}_{B_{std}}^{F_{i}} = \underbrace{\begin{bmatrix} \mathbf{I} & \Delta G_i \\ \mathbf{0} & 1 \end{bmatrix}}_{\mathbf{T}_{B_{std}}^{B_i}} \cdot \; \underbrace{\mathbf{T}_B^F(P_i)}_{\mathbf{T}_{B_i}^{F_i}}$$

展开即：

$$\mathbf{T}_{B_{std}}^{F_{i}} = \begin{bmatrix} \mathbf{R}(P_i) & \mathbf{t}(P_i) + \Delta G_i \\ \mathbf{0} & 1 \end{bmatrix}$$

> 物理含义：龙门架从 $G_{std}$ 移动到 $G_i$，基座平移了 $\Delta G_i$。在标准基座系 $\{B_{std}\}$ 中看，拍照点 $i$ 的法兰位置需要加上这个偏移量。

**第二步：计算相对于标准 Tag 的变换**

$$\boxed{\mathbf{R}_i = \big(\mathbf{T}_{B_{std}}^{T_{std}}\big)^{-1} \cdot \mathbf{T}_{B_{std}}^{F_{i}}}$$

$\mathbf{R}_i$（即配方中的 `relTransform`）表示：**从标准 Tag 坐标系到拍照点 $i$ 法兰的变换**，两者都在标准基座系 $\{B_{std}\}$ 中表达。

换言之：

$$\mathbf{T}_{B_{std}}^{F_i} = \mathbf{T}_{B_{std}}^{T_{std}} \cdot \mathbf{R}_i$$

---

## 6. 生产阶段

### 6.1 计算新位姿 — `computeNewPoses`

**输入：** $P_{cur}$、$(\mathbf{t}_{cur}, \mathbf{r}_{cur})$、$G_{cur}$

**目标：** 求出所有拍照点的新位姿（在标准基座系 $\{B_{std}\}$ 下）。

**Step A：计算当前 Tag 在当前基座系下的位姿**

$$\mathbf{T}_{B_{cur}}^{T_{cur}} = \mathbf{T}_B^F(P_{cur}) \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^T(\mathbf{t}_{cur}, \mathbf{r}_{cur})$$

**Step B：转换到标准基座系**

$$\Delta G_{cur} = G_{cur} - G_{std}$$

$$\boxed{\mathbf{T}_{B_{std}}^{T_{cur}} = \underbrace{\begin{bmatrix} \mathbf{I} & \Delta G_{cur} \\ \mathbf{0} & 1 \end{bmatrix}}_{\mathbf{T}_{B_{std}}^{B_{cur}}} \cdot \; \mathbf{T}_{B_{cur}}^{T_{cur}}}$$

> 这是关键步骤。Tag 是固定在空间中的物理物体。龙门架从 $G_{std}$ 移到 $G_{cur}$ 后，基座平移了 $\Delta G_{cur}$，因此 `computeTagInBase` 算出的 $\mathbf{T}_{B_{cur}}^{T_{cur}}$ 是在当前基座系下的。乘以 $\mathbf{T}_{B_{std}}^{B_{cur}}$ 将其"还原"到标准基座系。

**Step C：偏差传播到各拍照点**

对每个拍照点 $i$，利用标准 Tag 的"锚点"作用：

$$\boxed{\mathbf{T}_{B_{std}}^{F_{i}^{new}} = \mathbf{T}_{B_{std}}^{T_{cur}} \cdot \mathbf{R}_i}$$

展开完整的传播链：

$$\mathbf{T}_{B_{std}}^{F_{i}^{new}} = \underbrace{\mathbf{T}_{B_{std}}^{B_{cur}} \cdot \mathbf{T}_{B_{cur}}^{T_{cur}}}_{\text{当前Tag在标准基座系}} \cdot \; \underbrace{\big(\mathbf{T}_{B_{std}}^{T_{std}}\big)^{-1} \cdot \mathbf{T}_{B_{std}}^{F_i}}_{\mathbf{R}_i}$$

转为位姿：

$$P_i^{new} = \text{MatrixToPose}\big(\mathbf{T}_{B_{std}}^{F_{i}^{new}}\big)$$

此时 $P_i^{new}$ 是**标准基座系** $\{B_{std}\}$ 下的位姿。

### 6.2 龙门架调整 — `adjustForGantry`

拍照点 $i$ 的龙门架应移动到示教时的位置 $G_i$（否则相机视野不对）。此时机械臂需要 $\{B_i\}$ 下的位姿，而不是 $\{B_{std}\}$ 下的。

由式 4，从 $\{B_{std}\}$ 到 $\{B_i\}$ 的坐标转换：

$$\mathbf{p}^{B_i} = \mathbf{p}^{B_{std}} - (G_i - G_{std})$$

龙门架仅平移无旋转，旋转部分不变：

$$\boxed{\begin{aligned} x_{target} &= x_{std} - (G_{x,i} - G_{x,std}) \\ y_{target} &= y_{std} - (G_{y,i} - G_{y,std}) \\ z_{target} &= z_{std} - (G_{z,i} - G_{z,std}) \\ (r_x, r_y, r_z)_{target} &= (r_x, r_y, r_z)_{std} \end{aligned}}$$

---

## 7. 完整变换链总览

```
                    示教 (Teaching)
                    ═══════════════

  P_std ──→ T_{B_std}^{F_std} ──┐
  T_F^C ────────────────────────→ T_{B_std}^{T_std}  (标准Tag在基座系)
  (t_std, r_std) ──→ T_C^{T_std}┘
                                         │
  P_i ──→ T_{B_i}^{F_i} ──┐             │
  ΔG_i ──→ T_{B_std}^{B_i} ┤ T_{B_std}^{F_i} ──→ R_i = (T_{B_std}^{T_std})⁻¹ · T_{B_std}^{F_i}
                           ┘


                    生产 (Production)
                    ═════════════════

  P_cur ──→ T_{B_cur}^{F_cur} ──┐
  T_F^C ────────────────────────→ T_{B_cur}^{T_cur} ──┐
  (t_cur, r_cur) ──→ T_C^{T_cur}┘                      │
                                                        ├─→ T_{B_std}^{T_cur}
  ΔG_cur ──→ T_{B_std}^{B_cur} ────────────────────────┘         │
                                                                  │
  R_i ────────────────────────────────────────────────────────────┤
                                                                  ↓
                                            T_{B_std}^{F_i_new} = T_{B_std}^{T_cur} · R_i
                                                                  │
                                            ΔG_i = G_i - G_std ───┤
                                                                  ↓
                                            P_i_final = adjustForGantry(P_i_new)
```

---

## 8. 数值验算

使用单元测试 `testFullWorkflow` (Test 6) 的数据。

### 8.1 给定条件

- 手眼标定：$\mathbf{T}_F^C = [\mathbf{I} \mid (0, 0, 150)]$，仅 Z 方向偏移 150 mm
- $P_{std} = (400, 200, 500, 0°, -90°, 0°)$
- $\mathbf{t}_{std} = (0, 0, 0.5)\ \text{m}$，$\mathbf{r}_{std} = (0, 0, 0)$
- $G_{std} = (500, 300, 200)$

**欧拉角说明：** $(r_x, r_y, r_z) = (0°, -90°, 0°)$ 表示绕 Y 轴旋转 $-90°$：

$$\mathbf{R} = R_z(0°) \cdot R_y(-90°) \cdot R_x(0°) = R_y(-90°) = \begin{bmatrix} 0 & 0 & -1 \\ 0 & 1 & 0 \\ 1 & 0 & 0 \end{bmatrix}$$

### 8.2 标准 Tag 位姿

$$\mathbf{T}_{B_{std}}^{F_{std}} = \begin{bmatrix} 0 & 0 & -1 & 400 \\ 0 & 1 & 0 & 200 \\ 1 & 0 & 0 & 500 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$$\mathbf{T}_F^C \cdot \mathbf{T}_C^{T_{std}} = \begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 650 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$$\mathbf{T}_{B_{std}}^{T_{std}} = \mathbf{T}_{B_{std}}^{F_{std}} \cdot (\mathbf{T}_F^C \cdot \mathbf{T}_C^{T_{std}}) = \begin{bmatrix} 0 & 0 & -1 & -250 \\ 0 & 1 & 0 & 200 \\ 1 & 0 & 0 & 500 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

> Tag 在标准基座系中位于 $(-250, 200, 500)$ mm。

其逆矩阵：

$$(\mathbf{T}_{B_{std}}^{T_{std}})^{-1} = \begin{bmatrix} 0 & 0 & 1 & -500 \\ 0 & 1 & 0 & -200 \\ -1 & 0 & 0 & -250 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

### 8.3 拍照点的相对变换

**拍照点 1 (Front)：** $P_1 = (450, 200, 480, 0°, -90°, 0°)$，$G_1 = (520, 300, 180)$

$$\Delta G_1 = (20, 0, -20)$$

$$\mathbf{T}_{B_{std}}^{F_1} = [\mathbf{I} \mid \Delta G_1] \cdot \mathbf{T}_{B_1}^{F_1}(P_1) = \begin{bmatrix} 0 & 0 & -1 & 470 \\ 0 & 1 & 0 & 200 \\ 1 & 0 & 0 & 460 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

$$\mathbf{R}_1 = (\mathbf{T}_{B_{std}}^{T_{std}})^{-1} \cdot \mathbf{T}_{B_{std}}^{F_1} = \begin{bmatrix} 1 & 0 & 0 & -40 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & -720 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

> 法兰 $F_1$ 相对于标准 Tag 的偏移为 $(-40, 0, -720)$ mm。

**拍照点 2 (Side)：** $P_2 = (350, 250, 480, 0°, -90°, 0°)$，$G_2 = (480, 350, 200)$

$$\mathbf{R}_2 = [\mathbf{I} \mid (-20, 100, -580)]$$

**拍照点 3 (Top)：** $P_3 = (400, 200, 350, 0°, -90°, 0°)$，$G_3 = (500, 300, 200)$

$$\mathbf{R}_3 = [\mathbf{I} \mid (-150, 0, -650)]$$

### 8.4 生产阶段（龙门架未移动）

$P_{cur} = (405, 203, 500, 0°, -90°, 0°)$，$G_{cur} = G_{std} = (500, 300, 200)$

$\mathbf{t}_{cur} = (0.01, 0.005, 0.5)\ \text{m} = (10, 5, 500)\ \text{mm}$，$\mathbf{r}_{cur} = (0, 0, 0.01)\ \text{rad}$

$\mathbf{R}_{cam}^{tag} = \text{Rodrigues}(0, 0, 0.01) = R_z(0.01) = \begin{bmatrix} \cos 0.01 & -\sin 0.01 & 0 \\ \sin 0.01 & \cos 0.01 & 0 \\ 0 & 0 & 1 \end{bmatrix}$

$$\mathbf{T}_{B_{cur}}^{T_{cur}} = \mathbf{T}_{B_{std}}^{T_{cur}} = \begin{bmatrix} 0 & 0 & -1 & -245 \\ \sin 0.01 & \cos 0.01 & 0 & 208 \\ \cos 0.01 & -\sin 0.01 & 0 & 510 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

偏差传播到各拍照点：

$$\mathbf{T}_{B_{std}}^{F_1^{new}} = \mathbf{T}_{B_{std}}^{T_{cur}} \cdot \mathbf{R}_1$$

$$\begin{aligned} \mathbf{t}_1^{new} &= \mathbf{R}_{base}^{tag} \cdot (-40, 0, -720)^T + (-245, 208, 510)^T \\ &= (720,\ -40\sin 0.01,\ -40\cos 0.01)^T + (-245, 208, 510)^T \\ &= (475.0,\ 207.6,\ 470.0)^T \end{aligned}$$

$$P_1^{new} = (475.0,\ 207.6,\ 470.0,\ 0°, -90°, 0°) \quad \text{(标准基座系)}$$

同理：

$$P_2^{new} = (335.0,\ 307.8,\ 489.0,\ 0°, -90°, 0°)$$

$$P_3^{new} = (405.0,\ 206.5,\ 360.0,\ 0°, -90°, 0°)$$

### 8.5 龙门架调整

拍照点 1 目标龙门架 $G_1 = (520, 300, 180)$：

$$\begin{aligned} x_{final} &= 475.0 - (520 - 500) = 455.0 \\ y_{final} &= 207.6 - (300 - 300) = 207.6 \\ z_{final} &= 470.0 - (180 - 200) = 490.0 \end{aligned}$$

$$P_1^{final} = (455.0,\ 207.6,\ 490.0,\ 0°, -90°, 0°)$$

拍照点 2 目标龙门架 $G_2 = (480, 350, 200)$：

$$P_2^{final} = (355.0,\ 257.8,\ 489.0,\ 0°, -90°, 0°)$$

拍照点 3 目标龙门架 $G_3 = G_{std} = (500, 300, 200)$（无需调整）：

$$P_3^{final} = (405.0,\ 206.5,\ 360.0,\ 0°, -90°, 0°)$$

> 以上数值与 `test_deviation_corrector.cpp` 实际输出吻合（误差 $< 0.01$ mm）。

---

## 9. 特殊情况验证

### 9.1 仅机械臂移动（龙门架不动）

$G_{cur} = G_{std}$，则 $\Delta G_{cur} = 0$，$\mathbf{T}_{B_{std}}^{B_{cur}} = \mathbf{I}$。

$$\mathbf{T}_{B_{std}}^{T_{cur}} = \mathbf{T}_{B_{cur}}^{T_{cur}}$$

偏差传播仅由 Tag 检测的差异驱动。**✓ 正确。**

### 9.2 仅龙门架移动（机械臂不动）

机械臂不动（相对于当前基座），即 $P_{cur} = P_{std}$（数值相同），但 $G_{cur} \neq G_{std}$。

机械臂虽然没动，但因为基座移动了，Tag 在基座系下的位姿变了：

$$\mathbf{T}_{B_{cur}}^{F_{cur}} = \mathbf{T}_B^F(P_{std}) \quad \text{（数值相同）}$$

$$\mathbf{T}_{C}^{T_{cur}} = \mathbf{T}_C^{T_{std}} \quad \text{（Tag未动，相机未动）}$$

$$\mathbf{T}_{B_{cur}}^{T_{cur}} = \mathbf{T}_B^F(P_{std}) \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^{T_{std}}$$

此矩阵的平移部分与示教时 $\mathbf{T}_{B_{std}}^{T_{std}}$ 的平移部分相同。但龙门架偏移了 $\Delta G_{cur}$：

$$\mathbf{T}_{B_{std}}^{T_{cur}} = \begin{bmatrix} \mathbf{I} & \Delta G_{cur} \\ \mathbf{0} & 1 \end{bmatrix} \cdot \mathbf{T}_{B_{cur}}^{T_{cur}} = \begin{bmatrix} \mathbf{R} & \mathbf{t} + \Delta G_{cur} \\ \mathbf{0} & 1 \end{bmatrix}$$

传播后拍照点的新位姿平移部分会加上 $\Delta G_{cur}$，再经 `adjustForGantry` 减去各点的 $\Delta G_i$。最终龙门架的移动被正确补偿。**✓ 正确。**
