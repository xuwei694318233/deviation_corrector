# DeviationCorrector — 视觉伺服纠偏算法库

基于 AGV 统一偏移模型的工业机器人视觉伺服纠偏动态链接库 (DLL)。

## 核心模型

AGV 承载所有目标点（Pack / Tag）做刚性运动。AGV 停车偏差 ΔT 通过标准 Tag 检测获得，所有拍照点共享同一个 ΔT。

```
示教:  拍照点 i 记录机械臂位姿 P_i + 龙门架 G_i（帧转换用）
生产:  拍标准 Tag → 算 ΔT → 每点 P_i_new = [G_std→G_i]·ΔT·[G_i→G_std]·P_i
```

## 项目结构

```
├── include/deviation_corrector.hpp   # 头文件
├── src/deviation_corrector.cpp       # 实现
├── test/test_deviation_corrector.cpp # 单元测试
├── examples/                         # 使用示例
├── docs/                             # 数学推导文档
├── thirdparty/ (eigen, json)
├── CMakeLists.txt
└── README.md
```

## 依赖

| 依赖             | 版本   |
| ---------------- | ------ |
| Eigen3           | ≥ 3.4  |
| nlohmann/json    | 3.12.0 |
| CMake            | ≥ 3.14 |
| C++17            | —      |

## 快速构建

```
build.bat release   # Release
build.bat debug     # Debug
build.bat test      # 运行测试
```

## API

```cpp
MultiPointServo servo;
servo.setHandEyeCalibration(tFlangeCam);

// 示教
servo.startTeaching("recipe");
servo.recordStandardPoint(stdPose, stdTag, gantryX, gantryY, gantryZ);
servo.addPhotoPoint("点1", pose1, gantryX1, gantryY1, gantryZ1);
servo.addPhotoPoint("点2", pose2, gantryX2, gantryY2, gantryZ2);
servo.finishTeaching();
servo.saveRecipe("recipe.json");

// 生产
auto newPoses = servo.computeNewPoses(curPose, curTag, curX, curY, curZ);
```
