# DeviationCorrector - 视觉伺服纠偏算法库

工业机器人视觉伺服纠偏**动态链接库 (DLL)**，基于手眼标定的6自由度位姿纠偏算法，支持单点纠偏、多点位偏差传播，以及龙门架基座偏移补偿。

## 核心功能

- **单点纠偏**: 根据视觉检测偏差计算目标位姿
- **多点位偏差传播**: 一次检测自动更新所有拍照点位姿
- **龙门架基座补偿**: 自动补偿龙门架移动导致的基座坐标系偏移
- **配方持久化**: JSON格式配方保存/加载
- **纯C++接口**: 类接口设计，支持复制和移动语义

## 项目结构

```
deviation_corrector/
├── include/deviation_corrector.hpp    # 头文件
├── src/deviation_corrector.cpp        # 实现
├── test/test_deviation_corrector.cpp  # 单元测试 (10项)
├── examples/                          # 使用示例
├── thirdparty/
│   ├── eigen/                         # Eigen3 (git submodule)
│   └── json/json.hpp                  # nlohmann/json 3.12.0
├── CMakeLists.txt
└── README.md
```

## 依赖

| 依赖 | 版本 | 说明 |
|---|---|---|
| Eigen3 | ≥3.4 | 矩阵运算 (git submodule) |
| nlohmann/json | 3.12.0 | JSON解析 (内置) |
| CMake | ≥3.14 | 构建系统 |
| C++17 | - | 编译标准 |

## 快速构建

```powershell
# Debug
.\build.ps1 debug

# Release
.\build.ps1 release

# 运行测试
.\build.ps1 test
```

## 使用指南

### 单点纠偏

```cpp
#include "deviation_corrector.hpp"
using namespace vision_servo;

DeviationCorrector corrector;

// 设置手眼标定 (法兰→相机, 4x4齐次矩阵, mm单位)
Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
tFc(2, 3) = 150.0;
corrector.setHandEyeCalibration(tFc);

// 当前位姿
Pose6D curPose(500, 300, 400, 180, 0, 0);

// 视觉偏差 (相机坐标系, mm/deg)
auto dev = DeviationResult::xyPlane(-30, 10, 2);

// 计算纠偏目标
Pose6D target = corrector.calculateCorrection(curPose, dev);
```

### 多点位示教+生产 (含龙门架)

```cpp
MultiPointServo servo;
servo.setHandEyeCalibration(tFc);

// ===== 示教阶段 =====
servo.startTeaching("配方1");

// 记录标准点
servo.recordStandardPoint(stdPose, stdTag,
    500.0f, 300.0f, 200.0f);  // 龙门架XYZ (mm)

// 添加拍照点 (龙门架可不同位置)
servo.addPhotoPoint("正面", pose1, 520.0f, 300.0f, 180.0f);
servo.addPhotoPoint("侧面", pose2, 480.0f, 350.0f, 200.0f);

servo.finishTeaching();
servo.saveRecipe("recipe.json");

// ===== 生产阶段 =====
servo.loadRecipe("recipe.json");

auto newPoses = servo.computeNewPoses(curPose, newTag, 500.0f, 300.0f, 200.0f);

// 将标准基座系的位姿转换到目标龙门架位置
for (size_t i = 0; i < newPoses.size(); ++i)
{
    Pose6D adjusted = MultiPointServo::adjustForGantry(
        newPoses[i].second,
        500.0f, 300.0f, 200.0f,           // 标准点龙门架
        recipe.photoPoints[i].gantryX,     // 目标龙门架
        recipe.photoPoints[i].gantryY,
        recipe.photoPoints[i].gantryZ);

    robot.moveTo(adjusted);
    camera.capture();
}
```

## 数学原理

### 坐标变换链

```
T_base_tag = T_base_flange @ T_flange_cam @ T_cam_tag
```

### 龙门架基座补偿

```
T_std_flange_i = [I | G_i - G_std] @ T_base_i_flange_i
rel_transform_i = inv(T_base_tag_std) @ T_std_flange_i
```

### 偏差传播

```
T_base_flange_new = T_base_tag_new @ rel_transform_i
```

### 龙门架坐标调整

```
pose_target.x = pose_std_frame.x - (targetGantryX - stdGantryX)
pose_target.y = pose_std_frame.y - (targetGantryY - stdGantryY)
pose_target.z = pose_std_frame.z - (targetGantryZ - stdGantryZ)
```

## 接口说明

### DeviationCorrector

| 方法 | 功能 |
|---|---|
| `setHandEyeCalibration()` | 设置手眼标定矩阵 |
| `getHandEyeCalibration()` | 获取手眼标定矩阵 |
| `calculateCorrection()` | 单点纠偏计算 |
| `computeTagInBase()` | 计算Tag在基座系中位姿 |
| `propagateDeviation()` | 偏差传播 |
| `loadHandEyeFromFile()` | 加载手眼标定JSON |
| `poseToMatrix()` / `matrixToPose()` | 位姿↔矩阵转换 |

### MultiPointServo

| 方法 | 功能 |
|---|---|
| `startTeaching()` | 开始示教 |
| `recordStandardPoint()` | 记录标准点 (含龙门架位置) |
| `addPhotoPoint()` | 添加拍照点 (含龙门架位置) |
| `finishTeaching()` | 完成示教, 计算变换 |
| `computeNewPoses()` | 计算补偿位姿 (标准基座系) |
| `adjustForGantry()` | 龙门架坐标系转换 (static) |
| `saveRecipe()` / `loadRecipe()` | JSON配方持久化 |

## 配方JSON格式

```json
{
    "id": "recipe_1716153600",
    "name": "配方1",
    "createdTime": 1716153600.0,
    "stdRobotPose": [400.0, 200.0, 500.0, 0.0, -90.0, 0.0],
    "stdGantryX": 500.0,
    "stdGantryY": 300.0,
    "stdGantryZ": 200.0,
    "tBaseTagStd": [[...], [...], [...], [...]],
    "photoPoints": [
        {
            "name": "正面",
            "pose": [450.0, 200.0, 480.0, 0.0, -90.0, 0.0],
            "gantryX": 520.0,
            "gantryY": 300.0,
            "gantryZ": 180.0,
            "relTransform": [[...], [...], [...], [...]]
        }
    ]
}
```

## 手眼标定文件格式

支持两种JSON格式:

```json
// 格式1: 2D嵌套数组 (推荐)
{"T": [[1,0,0,50],[0,1,0,0],[0,0,1,100],[0,0,0,1]]}

// 格式2: 扁平数组, 行优先
{"result": [1,0,0,50, 0,1,0,0, 0,0,1,100, 0,0,0,1]}
```

## 坐标系约定

- **欧拉角顺序**: XYZ (外旋ZYX, R = Rz·Ry·Rx)
- **角度单位**: 度 (deg)
- **位置单位**: 毫米 (mm)
- **龙门架**: 仅平移, 无旋转

## License

MIT
