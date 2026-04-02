# DeviationCorrector - 视觉伺服纠偏算法库 (动态库)

## 📋 项目概述

DeviationCorrector 是一个用于工业机器人视觉伺服纠偏的 **动态链接库 (DLL)**。它实现了基于手眼标定的6自由度位姿纠偏算法，支持单点纠偏和多点位偏差传播。

### 库类型

| 平台    | 动态库                      | 导入库                    |
| ------- | --------------------------- | ------------------------- |
| Windows | `deviation_corrector.dll`   | `deviation_corrector.lib` |
| Linux   | `libdeviation_corrector.so` | -                         |

### 特点

- 动态链接库，便于外部程序调用
- 提供 C++ 类接口和 C 风格接口
- 跨语言调用支持 (通过 C 接口)
- PIMPL 模式隐藏实现细节

## 🎯 核心功能

- **单点纠偏**: 根据视觉检测偏差计算目标位姿
- **多点位偏差传播**: 一次检测自动更新所有拍照点位姿
- **手眼标定支持**: 支持 Eye-in-Hand 配置
- **完整6DOF**: 支持平移(X/Y/Z)和旋转(RX/RY/RZ)纠偏

## 🏗️ 项目结构

```
deviation_corrector/
├── include/
│   └── deviation_corrector.hpp    # 头文件 (接口定义)
├── src/
│   └── deviation_corrector.cpp    # 实现文件
├── test/
│   └── test_deviation_corrector.cpp # 单元测试
├── examples/
│   ├── example_single_correction.cpp # 单点纠偏示例
│   └── example_multi_point.cpp       # 多点位示例
├── thirdparty/
│   └── eigen/                     # Eigen3 库 (git submodule)
├── cmake/
│   └── deviation_corrector-config.cmake.in
├── build.bat                     # Windows 批处理构建脚本
├── build.ps1                     # PowerShell 构建脚本
├── .gitignore                    # Git忽略文件
├── .gitmodules                   # Git子模块配置
├── CMakeLists.txt
└── README.md
```

## 🔧 依赖项

| 依赖          | 版本    | 说明                    |
| ------------- | ------- | ----------------------- |
| Eigen3        | >= 3.4  | git submodule, 自动下载 |
| CMake         | >= 3.14 | 构建系统                |
| C++           | >= 17   | 编译标准                |
| Visual Studio | >= 2019 | Windows 编译器          |

## 🚀 快速开始

### 0. 克隆项目 (包含子模块)

```bash
# 克隆项目并自动初始化子模块
git clone --recursive https://github.com/your-repo/deviation_corrector.git

# 如果已经克隆，手动初始化子模块
git submodule update --init --recursive
```

### 方法1: 使用构建脚本 (推荐)

#### Windows 批处理脚本

```cmd
# 构建 Release 版本
build.bat release

# 构建 Debug 版本
build.bat debug

# 构建所有版本
build.bat all

# 清理构建目录
build.bat clean

# 仅配置项目
build.bat configure
```

#### PowerShell 脚本

```powershell
# 构建 Release 版本
./build.ps1 release

# 构建 Debug 版本
./build.ps1 debug

# 构建所有版本
./build.ps1 all

# 清理构建目录
./build.ps1 clean

# 仅运行测试
./build.ps1 test
```

### 方法2: 手动 CMake 构建

```powershell
# 创建构建目录
mkdir build
cd build

# 配置项目
cmake .. -G "Visual Studio 17 2022" -A x64

# 构建 Release 版本
cmake --build . --config Release

# 构建 Debug 版本
cmake --build . --config Debug

# 运行测试
ctest -C Release
```

### 方法3: 使用 Visual Studio

1. 打开 Visual Studio 2019/2022
2. 选择 "打开本地文件夹"
3. 选择 `deviation_corrector` 目录
4. Visual Studio 会自动检测 CMake 配置
5. 按 `Ctrl+Shift+B` 构建

### 方法4: 使用 CMake GUI

1. 打开 CMake GUI
2. **Where is the source code**: `w:/CATL/Eliteroboarm/deviation_corrector`
3. **Where to build the binaries**: `w:/CATL/Eliteroboarm/deviation_corrector/build`
4. 点击 **Configure** → 选择 `Visual Studio 17 2022` → 点击 **Finish**
5. 点击 **Generate**
6. 点击 **Open Project** 打开 `.sln` 文件

## 📖 使用指南

### 1. C++ 类接口

```cpp
#include "deviation_corrector.hpp"
using namespace vision_servo;

// 创建纠偏器
DeviationCorrector corrector;

// 设置手眼标定矩阵
Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
T_flange_cam(2, 3) = 150.0;  // 相机距离法兰150mm
corrector.setHandEyeCalibration(T_flange_cam);

// 当前位姿
Pose6D current_pose(500, 300, 400, 180, 0, 0);

// 视觉偏差
DeviationResult deviation(-30, 10, 0, 0, 0, 2);  // dx, dy, dz, drx, dry, drz

// 计算纠偏目标
Pose6D target = corrector.calculateCorrection(current_pose, deviation);

// 发送给机器人执行
robot.moveTo(target);
```

### 2. C 风格接口 (跨语言调用)

```c
#include "deviation_corrector.hpp"

// 创建纠偏器
DeviationCorrector* corrector = deviation_corrector_create();

// 设置手眼标定矩阵 (行优先, 16个double)
double hand_eye[16] = {1,0,0,0, 0,1,0,0, 0,0,1,150, 0,0,0,1};
deviation_corrector_set_hand_eye(corrector, hand_eye);

// 计算纠偏
double current_pose[6] = {500, 300, 400, 180, 0, 0};
double deviation[6] = {-30, 10, 0, 0, 0, 2};
double out_pose[6];
deviation_corrector_calculate(corrector, current_pose, deviation, out_pose);

// 销毁纠偏器
deviation_corrector_destroy(corrector);

// 获取版本
const char* ver = deviation_corrector_version();  // "DeviationCorrector v1.0.0"
```

### 3. 多点位纠偏 (C++ 接口)

```cpp
// 创建多点位控制器
MultiPointServo servo;

// ===== 示教阶段 =====
ServoRecipe recipe = servo.startTeaching("配方1");

// 记录标准点
Pose6D std_pose = robot.getPosition();
TagDetection tag = vision.detect();
servo.recordStandardPoint(std_pose, tag);

// 添加拍照点
servo.addPhotoPoint("点1", pose1);
servo.addPhotoPoint("点2", pose2);

// 完成示教
servo.finishTeaching();
servo.saveRecipe("recipe.json");

// ===== 生产阶段 =====
servo.loadRecipe("recipe.json");

// 检测当前Tag
TagDetection new_tag = vision.detect();
Pose6D cur_pose = robot.getPosition();

// 计算所有点位新位姿
auto new_poses = servo.computeNewPoses(cur_pose, new_tag);

// 执行
for (const auto& [name, pose] : new_poses) {
    robot.moveTo(pose);
    camera.capture();
}
```

## 📐 数学原理

### 核心公式

#### 单点纠偏

```
T_B_F_new = T_B_F_cur @ T_F_C @ T_dev @ T_F_C_inv

其中:
- T_B_F_cur: 当前机械臂位姿 (基座→法兰)
- T_F_C: 手眼标定矩阵 (法兰→相机)
- T_dev: 相机坐标系下的偏差变换
- T_B_F_new: 纠偏后的目标位姿
```

#### Tag在基座系中的位姿

```
T_base_tag = T_base_flange @ T_flange_cam @ T_cam_tag
```

#### 偏差传播

```
T_tag_flange_i = T_base_tag_std_inv @ T_base_flange_i  (示教时计算)
T_base_flange_i_new = T_base_tag_new @ T_tag_flange_i   (生产时计算)
```

### 坐标系约定

- **欧拉角顺序**: XYZ (外旋 ZYX)
- **角度单位**: 度 (degree)
- **位置单位**: 毫米 (mm)
- **旋转矩阵**: R = Rz(rz) · Ry(ry) · Rx(rx)

## 🔌 接口说明

### IDeviationCorrector (抽象接口)

| 方法                      | 功能                    |
| ------------------------- | ----------------------- |
| `setHandEyeCalibration()` | 设置手眼标定矩阵        |
| `getHandEyeCalibration()` | 获取手眼标定矩阵        |
| `calculateCorrection()`   | 计算纠偏后目标位姿      |
| `computeTagInBase()`      | 计算Tag在基座系中的位姿 |
| `propagateDeviation()`    | 偏差传播                |

### MultiPointServo (多点位控制器)

| 方法                    | 功能       |
| ----------------------- | ---------- |
| `startTeaching()`       | 开始示教   |
| `recordStandardPoint()` | 记录标准点 |
| `addPhotoPoint()`       | 添加拍照点 |
| `finishTeaching()`      | 完成示教   |
| `computeNewPoses()`     | 计算新位姿 |
| `saveRecipe()`          | 保存配方   |
| `loadRecipe()`          | 加载配方   |

### C 风格接口 (跨语言调用)

| 函数                                 | 功能             |
| ------------------------------------ | ---------------- |
| `deviation_corrector_create()`       | 创建纠偏器实例   |
| `deviation_corrector_destroy()`      | 销毁纠偏器实例   |
| `deviation_corrector_set_hand_eye()` | 设置手眼标定矩阵 |
| `deviation_corrector_calculate()`    | 计算纠偏         |
| `deviation_corrector_version()`      | 获取库版本号     |

## 🧪 运行测试

```powershell
# 使用构建脚本 (构建后自动运行测试)
./build.ps1 release

# 或手动运行
./build/test_release/test_deviation_corrector.exe
```

测试内容:

1. 位姿-矩阵转换测试
2. Rodrigues变换测试
3. 单点纠偏测试
4. Tag位姿计算测试
5. 多点位偏差传播测试
6. 完整示教-生产流程测试

## 📊 构建输出

构建完成后，输出目录结构：

```
build/
├── Release/
│   ├── deviation_corrector.dll    # 动态库
│   ├── deviation_corrector.lib    # 导入库
│   ├── test_deviation_corrector.exe
│   ├── example_single_correction.exe
│   └── example_multi_point.exe
├── Debug/
│   ├── deviation_corrector.dll
│   ├── deviation_corrector.lib
│   └── ...
└── ...
```

### 使用动态库

1. **编译时**: 链接 `deviation_corrector.lib`，包含头文件目录
2. **运行时**: 确保 `deviation_corrector.dll` 在可执行文件同目录或系统 PATH 中

```cmake
# 在你的 CMakeLists.txt 中
target_link_libraries(your_target PRIVATE deviation_corrector)
target_include_directories(your_target PRIVATE path/to/include)
```

## 🔄 与Python版本对比

| 特性     | C++      | Python   |
| -------- | -------- | -------- |
| 执行速度 | 快       | 慢       |
| 内存占用 | 低       | 高       |
| 部署难度 | 高       | 低       |
| 开发效率 | 中       | 高       |
| 适用场景 | 实时控制 | 原型开发 |

## 📝 注意事项

1. **手眼标定**: 使用前必须设置正确的手眼标定矩阵
2. **万向节锁**: 当 RY ≈ ±90° 时，欧拉角可能出现奇异，但旋转矩阵正确
3. **坐标系**: 确保机器人坐标系与标定时一致
4. **单位**: 位置用mm，角度用度

## 🛠️ 故障排除

### CMake 未找到

```
错误: 'cmake' 不是内部或外部命令
解决: 安装 CMake 并添加到系统 PATH
下载: https://cmake.org/download/
```

### Visual Studio 未找到

```
错误: Could not find Visual Studio
解决: 安装 Visual Studio 2019 或更高版本
确保安装了 "使用 C++ 的桌面开发" 工作负载
```

### Eigen3 未找到

```
错误: Could not find Eigen3
解决: 初始化 git 子模块
      git submodule update --init --recursive
      检查 thirdparty/eigen/Eigen 目录是否存在
```

## 📜 许可证

MIT License

## 🤝 贡献指南

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

## 📞 联系方式

如有问题或建议，请提交 Issue。
