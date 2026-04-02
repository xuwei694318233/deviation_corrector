# Thirdparty Libraries

此目录包含项目所需的第三方库，通过 **Git Submodule** 管理。

## Eigen3

Eigen 是一个高质量的 C++ 模板库，用于线性代数、矩阵、向量和数值计算。

### 初始化子模块

```bash
# 克隆项目时自动初始化
git clone --recursive https://github.com/your-repo/deviation_corrector.git

# 或在已克隆的项目中初始化
git submodule update --init --recursive
```

### 更新子模块

```bash
# 更新到最新版本
git submodule update --remote thirdparty/eigen

# 切换到特定版本 (如 3.4.0)
cd thirdparty/eigen
git checkout 3.4.0
cd ../..
git add thirdparty/eigen
```

### 使用说明

Eigen 是 header-only 库，无需编译。CMakeLists.txt 已配置为自动使用本地版本。

```cpp
#include <Eigen/Dense>

Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
Eigen::Vector3d vector(1.0, 2.0, 3.0);
```

### 许可证

Eigen 使用 **MPL2** (Mozilla Public License 2.0) 和 **LGPL** 双许可。
可自由用于商业和非商业项目。

### 官方链接

- 官网: https://eigen.tuxfamily.org/
- GitLab: https://gitlab.com/libeigen/eigen
- GitHub 镜像: https://github.com/libeigen/eigen

## 目录结构

```
thirdparty/
├── eigen/           # Eigen3 (git submodule)
│   ├── Eigen/       # 主要头文件
│   ├── cmake/       # CMake 配置
│   └── ...
└── README.md        # 本文件
```
