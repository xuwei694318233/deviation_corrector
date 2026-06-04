# 柔性拍摄纠偏 — 数学推导

## 1. 坐标系

| 符号          | 含义                                     |
| ------------- | ---------------------------------------- |
| $\{B_i\}$     | 龙门架位于 $G_i$ 处的机械臂基座系          |
| $\{F\}$       | 法兰坐标系（机械臂末端）                    |
| $\{C\}$       | 相机坐标系（通过法兰固定）                  |
| $\{T\}$       | Tag 坐标系（固定在 AGV 上，所有拍照点共享） |
| $G_i$         | 龙门架编码器读数 (mm)，纯平移无旋转         |

**核心事实**: Tag 和所有拍照点目标固定在 AGV 上，AGV 移动时一起做同一个刚性运动。

**龙门架**: 仅用于基座系帧转换（纯平移），不嵌入矩阵乘法链。

---

## 2. 基础公式

位姿 → 4×4 矩阵：

$$\mathbf{T}_B^F(P) = \begin{bmatrix} R_z R_y R_x & \mathbf{t} \\ \mathbf{0} & 1 \end{bmatrix}$$

Tag 检测 → 4×4 矩阵：

$$\mathbf{T}_C^T(\mathbf{t},\mathbf{r}) = \begin{bmatrix} \text{Rodrigues}(\mathbf{r}) & \mathbf{t} \cdot 1000 \\ \mathbf{0} & 1 \end{bmatrix}$$

Tag 在基座系中的位姿（变换链）：

$$\mathbf{T}_B^T = \mathbf{T}_B^F \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^T$$

基座系帧转换（纯平移）：

$$\mathbf{T}_{B_{std}}^{B_i} = \begin{bmatrix} \mathbf{I} & G_i - G_{std} \\ \mathbf{0} & 1 \end{bmatrix}$$

---

## 3. 示教阶段

### 3.1 标准 Tag

AGV 在标准位，龙门架在 $G_{std}$，机械臂在 $P_{std}$：

$$\boxed{\mathbf{T}_{B_{std}}^T = \mathbf{T}_B^F(P_{std}) \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^T(\mathbf{t}_{std}, \mathbf{r}_{std})}$$

$\mathbf{T}_{B_{std}}^T$ 即 `tBaseTagStd`，Tag 在标准基座系 $\{B_{std}\}$ 下的位姿。

### 3.2 拍照点

每个拍照点 $i$ 记录：

- 机械臂位姿 $P_i$（在 $\{B_i\}$ 系）
- 龙门架位置 $G_i$

法兰齐次矩阵：$\mathbf{T}_{B_i}^{F_i} = \mathbf{T}_B^F(P_i)$

---

## 4. 生产阶段

### 4.1 当前 Tag

AGV 停车有偏差，龙门架在 $G_{cur}$，机械臂在 $P_{cur}$：

$$\mathbf{T}_{B_{cur}}^T = \mathbf{T}_B^F(P_{cur}) \cdot \mathbf{T}_F^C \cdot \mathbf{T}_C^T(\mathbf{t}_{cur}, \mathbf{r}_{cur})$$

$\mathbf{T}_{B_{cur}}^T$ 在当前基座系 $\{B_{cur}\}$ 中。

### 4.2 帧转换到标准基座系

$$\boxed{\mathbf{T}_{B_{std}}^T = \mathbf{T}_{B_{std}}^{B_{cur}} \cdot \mathbf{T}_{B_{cur}}^T}$$

$$= \begin{bmatrix} \mathbf{I} & G_{cur} - G_{std} \\ \mathbf{0} & 1 \end{bmatrix} \cdot \mathbf{T}_{B_{cur}}^T$$

### 4.3 AGV 偏移变换 ΔT

$$\boxed{\Delta\mathbf{T} = \mathbf{T}_{B_{std}}^T \cdot \big(\mathbf{T}_{B_{std}}^T\big)^{-1}_{示教}}$$

ΔT 描述"示教时 Tag → 当前 Tag"在 $\{B_{std}\}$ 系中的刚性变换。

### 4.4 传播到拍照点 $i$

拍照点 $i$ 的法兰在 $\{B_i\}$ 系：$\mathbf{T}_{B_i}^{F_i}$

**① 转到 $\{B_{std}\}$ 系**

$$\mathbf{T}_{B_{std}}^{F_i} = \mathbf{T}_{B_{std}}^{B_i} \cdot \mathbf{T}_{B_i}^{F_i} = \begin{bmatrix} \mathbf{I} & G_i - G_{std} \\ \mathbf{0} & 1 \end{bmatrix} \cdot \mathbf{T}_{B_i}^{F_i}$$

**② 应用 AGV 偏移（左乘）**

$$\mathbf{T}_{B_{std}}^{F_i^{new}} = \Delta\mathbf{T} \cdot \mathbf{T}_{B_{std}}^{F_i}$$

**③ 转回 $\{B_i\}$ 系**

$$\mathbf{T}_{B_i}^{F_i^{new}} = \big(\mathbf{T}_{B_{std}}^{B_i}\big)^{-1} \cdot \mathbf{T}_{B_{std}}^{F_i^{new}} = \begin{bmatrix} \mathbf{I} & G_{std} - G_i \\ \mathbf{0} & 1 \end{bmatrix} \cdot \mathbf{T}_{B_{std}}^{F_i^{new}}$$

**完整公式**:

$$\boxed{\mathbf{T}_{B_i}^{F_i^{new}} = \begin{bmatrix} \mathbf{I} & G_{std} - G_i \\ \mathbf{0} & 1 \end{bmatrix} \cdot \Delta\mathbf{T} \cdot \begin{bmatrix} \mathbf{I} & G_i - G_{std} \\ \mathbf{0} & 1 \end{bmatrix} \cdot \mathbf{T}_{B_i}^{F_i}}$$

最终位姿：$P_i^{new} = \text{MatrixToPose}\big(\mathbf{T}_{B_i}^{F_i^{new}}\big)$

---

## 5. 验证

**所有点同龙门架** ($G_i = G_{cur} = G_{std}$)：帧转换矩阵退化为 $\mathbf{I}$

$$\mathbf{T}_{B_i}^{F_i^{new}} = \Delta\mathbf{T} \cdot \mathbf{T}_{B_i}^{F_i} \quad \checkmark$$

**AGV 未移动** ($\mathbf{T}_{B_{cur}}^T = \mathbf{T}_{B_{std}}^T$, $G_{cur} = G_{std}$)：

$$\mathbf{T}_{B_{std}}^T = \mathbf{T}_{B_{std}}^T \;\Rightarrow\; \Delta\mathbf{T} = \mathbf{I} \;\Rightarrow\; \mathbf{T}_{B_i}^{F_i^{new}} = \mathbf{T}_{B_i}^{F_i} \quad \checkmark$$

---

## 6. 代码对应

```cpp
// T_cur 在 G_cur 系 → 转到 G_std 系
Eigen::Matrix4d curToStd = Eigen::Matrix4d::Identity();
curToStd(0,3) = curGantryX - stdGantryX;
curToStd(1,3) = curGantryY - stdGantryY;
curToStd(2,3) = curGantryZ - stdGantryZ;
auto tBaseTagCurStd = curToStd * tBaseTagCur;   // T_{B_std}^T (当前)

// ΔT = T_{B_std}^T · (T_{B_std}^T)^{-1}_{示教}
auto deltaT = tBaseTagCurStd * tBaseTagStd.inverse();

for each photo point i:
    auto toStd = [I | G_i - G_std];            // 转到标准系
    auto tNew = toStd.inverse() * deltaT * toStd * poseToMatrix(P_i);
    P_i_new = matrixToPose(tNew);              // 在 G_i 系
```
