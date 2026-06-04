#pragma once

/**
 * @file deviation_corrector.hpp
 * @brief 视觉伺服纠偏算法库 — AGV 统一偏移模型 (动态库版本)
 *
 * 核心模型：AGV 承载所有目标点（Pack/Tag），AGV 停车偏差时，
 * 所有目标点随 AGV 做同一个刚性变换 ΔT。
 * 每个拍照点只需 P_i_new = P_i · ΔT 即可获得纠偏位姿。
 *
 * 坐标系约定:
 *   - 欧拉角顺序: XYZ (外旋 ZYX = 内旋 XYZ)
 *   - 角度单位: 度 (deg)
 *   - 位置单位: 毫米 (mm)
 *   - 旋转矩阵: R = Rz(rz) · Ry(ry) · Rx(rx)
 */

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <memory>
#include <cmath>

#if defined(_WIN32) || defined(__CYGWIN__)
    #ifdef DEVIATION_CORRECTOR_EXPORTS
        #define DEV_CORRECTOR_API __declspec(dllexport)
    #else
        #define DEV_CORRECTOR_API __declspec(dllimport)
    #endif
#else
    #define DEV_CORRECTOR_API __attribute__((visibility("default")))
#endif

namespace vision_servo
{

constexpr double kMmToM = 0.001;  ///< 毫米 → 米的换算因子

// ==================== 数据结构 ====================

/**
 * @brief 6 自由度位姿 (mm, deg)
 */
struct DEV_CORRECTOR_API Pose6D
{
    double x  = 0.0;  ///< X 坐标 (mm)
    double y  = 0.0;  ///< Y 坐标 (mm)
    double z  = 0.0;  ///< Z 坐标 (mm)
    double rx = 0.0;  ///< 绕 X 轴旋转 (deg)
    double ry = 0.0;  ///< 绕 Y 轴旋转 (deg)
    double rz = 0.0;  ///< 绕 Z 轴旋转 (deg)

    Pose6D() = default;
    Pose6D(double x_, double y_, double z_, double rx_, double ry_, double rz_)
        : x(x_), y(y_), z(z_), rx(rx_), ry(ry_), rz(rz_) {}

    /// @brief 转为 6 元素向量 [x, y, z, rx, ry, rz]
    std::vector<double> toVector() const { return {x, y, z, rx, ry, rz}; }

    /// @brief 从 6 元素向量创建
    static Pose6D fromVector(const std::vector<double>& v)
    {
        return (v.size() >= 6) ? Pose6D(v[0], v[1], v[2], v[3], v[4], v[5]) : Pose6D();
    }
};

/**
 * @brief 视觉偏差量（相机坐标系, mm / deg）
 */
struct DEV_CORRECTOR_API DeviationResult
{
    double dx  = 0.0;  ///< X 方向平移偏差 (mm)
    double dy  = 0.0;  ///< Y 方向平移偏差 (mm)
    double dz  = 0.0;  ///< Z 方向平移偏差 (mm)
    double drx = 0.0;  ///< 绕 X 轴旋转偏差 (deg)
    double dry = 0.0;  ///< 绕 Y 轴旋转偏差 (deg)
    double drz = 0.0;  ///< 绕 Z 轴旋转偏差 (deg)

    DeviationResult() = default;
    DeviationResult(double dx_, double dy_, double dz_, double drx_, double dry_, double drz_)
        : dx(dx_), dy(dy_), dz(dz_), drx(drx_), dry(dry_), drz(drz_) {}

    /// @brief 创建纯平移偏差
    static DeviationResult translation(double dx_, double dy_, double dz_)
    {
        return {dx_, dy_, dz_, 0.0, 0.0, 0.0};
    }

    /// @brief 创建 XY 平面偏差（平移 + 绕 Z 旋转）
    static DeviationResult xyPlane(double dx_, double dy_, double drz_)
    {
        return {dx_, dy_, 0.0, 0.0, 0.0, drz_};
    }
};

/**
 * @brief Tag 检测结果（相机坐标系）
 */
struct DEV_CORRECTOR_API TagDetection
{
    int id = 0;                                          ///< Tag ID
    Eigen::Vector3d tvec{Eigen::Vector3d::Zero()};      ///< 平移向量 (m)
    Eigen::Vector3d rvec{Eigen::Vector3d::Zero()};      ///< 旋转向量 Rodrigues (rad)
    Eigen::Vector3d euler{Eigen::Vector3d::Zero()};     ///< 欧拉角 XYZ (deg)
    Eigen::Vector2d center{Eigen::Vector2d::Zero()};    ///< Tag 像素中心
};

/**
 * @brief 拍照点位数据
 */
struct DEV_CORRECTOR_API PhotoPoint
{
    std::string name;                       ///< 点位名称
    Pose6D      pose;                       ///< 机械臂末端位姿 (mm/deg)
    float gantryX = 0.0f;                   ///< 龙门架 X (mm, 仅帧转换用)
    float gantryY = 0.0f;                   ///< 龙门架 Y (mm)
    float gantryZ = 0.0f;                   ///< 龙门架 Z (mm)
    std::string snapshotPath;               ///< 快照路径（可选）
    std::optional<TagDetection> tagData;    ///< Tag 检测数据（可选）
};

/**
 * @brief 视觉伺服配方
 */
struct DEV_CORRECTOR_API ServoRecipe
{
    std::string id;                         ///< 配方 ID
    std::string name;                       ///< 配方名称
    double      createdTime = 0.0;          ///< 创建时间戳
    std::string description;                ///< 描述

    Pose6D        stdRobotPose;             ///< 标准点机械臂位姿
    float         stdGantryX = 0.0f;        ///< 标准点龙门架 X (mm, 仅帧转换)
    float         stdGantryY = 0.0f;        ///< 标准点龙门架 Y (mm)
    float         stdGantryZ = 0.0f;        ///< 标准点龙门架 Z (mm)
    TagDetection  stdTagData;               ///< 标准点 Tag 检测数据
    Eigen::Matrix4d tBaseTagStd{Eigen::Matrix4d::Identity()};  ///< 标准 Tag 在基座系下的位姿

    std::vector<PhotoPoint> photoPoints;    ///< 拍照点位列表
    std::string             handEyeFile;    ///< 手眼标定文件路径（可选）
};

// ==================== DeviationCorrector ====================

/**
 * @brief 单点纠偏算法
 *
 * 提供位姿矩阵转换、Rodrigues、欧拉角等基础工具，
 * 以及基于手眼标定的 Tag 位姿计算和单点纠偏。
 */
class DEV_CORRECTOR_API DeviationCorrector
{
public:
    DeviationCorrector();
    ~DeviationCorrector();

    /**
     * @brief 设置手眼标定矩阵
     * @param tFlangeCam 法兰 → 相机的 4×4 齐次矩阵
     */
    void setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam);

    /**
     * @brief 获取手眼标定矩阵
     * @return 法兰 → 相机的 4×4 齐次矩阵
     */
    Eigen::Matrix4d getHandEyeCalibration() const;

    /**
     * @brief 单点纠偏：根据视觉偏差计算目标法兰位姿
     * @param currentPose 当前机械臂位姿 (mm/deg)
     * @param deviation   视觉偏差 (相机坐标系, mm/deg)
     * @return 纠偏后目标位姿
     */
    Pose6D calculateCorrection(const Pose6D& currentPose, const DeviationResult& deviation);

    /**
     * @brief 计算 Tag 在机械臂基座系下的位姿
     * @param robotPose 机械臂当前位姿 (mm/deg)
     * @param tagTvec   Tag 在相机系下的平移向量 (m)
     * @param tagRvec   Tag 在相机系下的旋转向量 Rodrigues (rad)
     * @return Tag 在基座系下的 4×4 齐次矩阵 (mm)
     */
    Eigen::Matrix4d computeTagInBase(const Pose6D& robotPose,
                                      const Eigen::Vector3d& tagTvec,
                                      const Eigen::Vector3d& tagRvec);

    /**
     * @brief 从 JSON 文件加载手眼标定矩阵
     *
     * 支持两种格式:
     *   - {"T": [[4x4 二维数组]]}
     *   - {"result": [16 元素扁平数组, 行优先]}
     *
     * @param filepath JSON 文件路径
     * @return 是否加载成功
     */
    bool loadHandEyeFromFile(const std::string& filepath);

    /**
     * @brief 6DOF 位姿 → 4×4 齐次矩阵
     * @param pose     位姿 (mm/deg)
     * @param isDegree 角度是否为度（默认 true）
     * @return 4×4 齐次矩阵
     */
    static Eigen::Matrix4d poseToMatrix(const Pose6D& pose, bool isDegree = true);

    /**
     * @brief 4×4 齐次矩阵 → 6DOF 位姿
     * @param matrix   4×4 齐次矩阵
     * @param toDegree 是否输出度数（默认 true）
     * @return 位姿 (mm, deg)
     */
    static Pose6D matrixToPose(const Eigen::Matrix4d& matrix, bool toDegree = true);

    /**
     * @brief Rodrigues 旋转向量 → 3×3 旋转矩阵
     * @param rvec 旋转向量 (rad, 模长 = 旋转角度)
     * @return 3×3 旋转矩阵
     */
    static Eigen::Matrix3d rodriguesToMatrix(const Eigen::Vector3d& rvec);

    /**
     * @brief 3×3 旋转矩阵 → XYZ 欧拉角
     * @param r        3×3 旋转矩阵
     * @param toDegree 是否输出度数
     * @return 欧拉角 (rx, ry, rz)
     */
    static Eigen::Vector3d matrixToEulerXyz(const Eigen::Matrix3d& r, bool toDegree = true);

    /**
     * @brief XYZ 欧拉角 → 3×3 旋转矩阵 R = Rz·Ry·Rx
     * @param euler    欧拉角 (rx, ry, rz)
     * @param isDegree 是否为度数
     * @return 3×3 旋转矩阵
     */
    static Eigen::Matrix3d eulerXyzToMatrix(const Eigen::Vector3d& euler, bool isDegree = true);

private:
    struct Impl;
    std::unique_ptr<Impl> pImpl_;  ///< PIMPL 实现
};

// ==================== MultiPointServo ====================

/**
 * @brief 多点位视觉伺服控制器
 *
 * 管理「示教 → 生产」完整工作流。
 *
 * 示教阶段：记录标准 Tag + 各拍照点机械臂位姿。
 * 生产阶段：拍当前 Tag → 算 ΔT = T_std⁻¹ · T_cur → 每个 P_i_new = P_i · ΔT。
 *
 * 龙门架固定在标准点位置（拍照点不改龙门架坐标，仅改机械臂）。
 */
class DEV_CORRECTOR_API MultiPointServo
{
public:
    MultiPointServo();
    explicit MultiPointServo(const std::string& handEyeFile);
    ~MultiPointServo();

    // ========== 示教阶段 ==========

    /**
     * @brief 开始新的示教流程
     * @param name 配方名称（为空则自动生成时间戳名称）
     * @return 新创建的空白配方
     */
    ServoRecipe startTeaching(const std::string& name = "");

    /**
     * @brief 记录标准点（视觉基准点）
     * @param robotPose 机械臂当前位姿 (mm/deg)
     * @param tagResult Tag 检测结果
     * @param gantryX 龙门架 X (mm, 仅帧转换)
     * @param gantryY 龙门架 Y (mm)
     * @param gantryZ 龙门架 Z (mm)
     * @return 是否成功
     */
    bool recordStandardPoint(const Pose6D& robotPose, const TagDetection& tagResult,
        float gantryX, float gantryY, float gantryZ);

    /**
     * @brief 添加拍照点位
     * @param name      点位名称
     * @param robotPose 机械臂位姿 (mm/deg)
     * @param gantryX 龙门架 X (mm, 仅帧转换)
     * @param gantryY 龙门架 Y (mm)
     * @param gantryZ 龙门架 Z (mm)
     * @return 当前拍照点总数
     */
    int addPhotoPoint(const std::string& name, const Pose6D& robotPose,
        float gantryX, float gantryY, float gantryZ);

    /**
     * @brief 完成示教
     * @return 填充完成的配方
     */
    ServoRecipe finishTeaching();

    // ========== 生产阶段 ==========

    /**
     * @brief 计算 AGV 偏移量 ΔT，传播到所有拍照点
     *
     * 算法: ΔT = T_cur_std · T_std⁻¹（左乘），龙门架帧转换用纯平移在乘法链外。
     *
     * @param robotPose  当前机械臂位姿 (mm/deg)
     * @param tagResult  当前 Tag 检测结果
     * @param curGantryX 当前龙门架 X (mm, 仅帧转换)
     * @param curGantryY 当前龙门架 Y (mm)
     * @param curGantryZ 当前龙门架 Z (mm)
     * @return 拍照点新位姿列表 [(名称, 在各自 G_i 系下的位姿), ...]
     */
    std::vector<std::pair<std::string, Pose6D>> computeNewPoses(
        const Pose6D& robotPose, const TagDetection& tagResult,
        float curGantryX, float curGantryY, float curGantryZ);

    // ========== 持久化 ==========

    /**
     * @brief 从 JSON 文件加载配方
     * @param filepath 配方文件路径
     * @return 是否成功
     */
    bool loadRecipe(const std::string& filepath);

    /**
     * @brief 保存配方到 JSON 文件
     * @param filepath 保存路径
     * @return 是否成功
     */
    bool saveRecipe(const std::string& filepath) const;

    // ========== 手眼标定 ==========

    /**
     * @brief 设置手眼标定矩阵
     * @param tFlangeCam 法兰 → 相机的 4×4 齐次矩阵
     */
    void setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam);

    /**
     * @brief 从 JSON 文件加载手眼标定矩阵
     * @param filepath JSON 文件路径
     * @return 是否成功
     */
    bool loadHandEyeFromFile(const std::string& filepath);

    /**
     * @brief 获取当前配方（只读）
     * @return 配方引用
     */
    const ServoRecipe& getCurrentRecipe() const;

private:
    struct Impl;
    std::unique_ptr<Impl> pImpl_;  ///< PIMPL 实现
};

} // namespace vision_servo
