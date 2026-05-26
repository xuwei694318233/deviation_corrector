#pragma once

/**
 * @file deviation_corrector.hpp
 * @brief 视觉伺服纠偏算法库 - 标准化接口 (动态库版本)
 *
 * 提供工业机器人视觉伺服纠偏的核心算法实现。
 * 支持单点纠偏、多点位偏差传播，以及龙门架基座偏移补偿。
 *
 * 坐标系约定:
 *   - 欧拉角顺序: XYZ (外旋 ZYX = 内旋 XYZ)
 *   - 角度单位: 度 (deg)
 *   - 位置单位: 毫米 (mm)
 *   - 旋转矩阵: R = Rz(rz) * Ry(ry) * Rx(rx)
 */

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <optional>
#include <cmath>

// ==================== 动态库导出宏 ====================
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

// ==================== 常量 ====================

/// 毫米与米之间的换算因子
constexpr double kMmToM = 0.001;

// ==================== 数据结构定义 ====================

/**
 * @brief 6自由度位姿 (mm, deg)
 */
struct DEV_CORRECTOR_API Pose6D
{
    double x  = 0.0; ///< X位置 (mm)
    double y  = 0.0; ///< Y位置 (mm)
    double z  = 0.0; ///< Z位置 (mm)
    double rx = 0.0; ///< 绕X轴旋转角度 (deg)
    double ry = 0.0; ///< 绕Y轴旋转角度 (deg)
    double rz = 0.0; ///< 绕Z轴旋转角度 (deg)

    Pose6D() = default;
    Pose6D(double x_, double y_, double z_, double rx_, double ry_, double rz_)
        : x(x_), y(y_), z(z_), rx(rx_), ry(ry_), rz(rz_)
    {
    }

    /// 转换为向量 [x, y, z, rx, ry, rz]
    std::vector<double> toVector() const
    {
        return {x, y, z, rx, ry, rz};
    }

    /// 从向量创建 (至少6个元素)
    static Pose6D fromVector(const std::vector<double>& v)
    {
        if (v.size() >= 6)
        {
            return Pose6D(v[0], v[1], v[2], v[3], v[4], v[5]);
        }
        return Pose6D();
    }
};

/**
 * @brief 视觉偏差量 (相机坐标系, mm / deg)
 */
struct DEV_CORRECTOR_API DeviationResult
{
    double dx  = 0.0; ///< X方向偏差 (mm)
    double dy  = 0.0; ///< Y方向偏差 (mm)
    double dz  = 0.0; ///< Z方向偏差 (mm)
    double drx = 0.0; ///< 绕X轴旋转偏差 (deg)
    double dry = 0.0; ///< 绕Y轴旋转偏差 (deg)
    double drz = 0.0; ///< 绕Z轴旋转偏差 (deg)

    DeviationResult() = default;
    DeviationResult(double dx_, double dy_, double dz_, double drx_, double dry_, double drz_)
        : dx(dx_), dy(dy_), dz(dz_), drx(drx_), dry(dry_), drz(drz_)
    {
    }

    /// 创建仅平移偏差 (3DOF)
    static DeviationResult translation(double dx_, double dy_, double dz_)
    {
        return DeviationResult(dx_, dy_, dz_, 0.0, 0.0, 0.0);
    }

    /// 创建XY平面纠偏 (常用场景, 3DOF)
    static DeviationResult xyPlane(double dx_, double dy_, double drz_)
    {
        return DeviationResult(dx_, dy_, 0.0, 0.0, 0.0, drz_);
    }
};

/**
 * @brief Tag检测结果 (相机坐标系)
 */
struct DEV_CORRECTOR_API TagDetection
{
    int    id     = 0;                    ///< Tag ID
    Eigen::Vector3d tvec;                 ///< 平移向量 (相机坐标系, 米)
    Eigen::Vector3d rvec;                 ///< 旋转向量 (Rodrigues, 弧度)
    Eigen::Vector3d euler;                ///< 欧拉角 XYZ (deg)
    Eigen::Vector2d center;               ///< 像素中心坐标

    TagDetection()
        : tvec(Eigen::Vector3d::Zero())
        , rvec(Eigen::Vector3d::Zero())
        , euler(Eigen::Vector3d::Zero())
        , center(Eigen::Vector2d::Zero())
    {
    }
};

/**
 * @brief 拍照点位数据
 *
 * 包含机械臂位姿以及记录时的龙门架位置。
 * 龙门架位置用于补偿因基座移动引起的坐标系偏移。
 */
struct DEV_CORRECTOR_API PhotoPoint
{
    std::string   name;                      ///< 点位名称
    Pose6D        pose;                      ///< 机械臂末端位姿 (基座坐标系, mm/deg)
    float         gantryX = 0.0f;            ///< 记录时龙门架X位置 (mm)
    float         gantryY = 0.0f;            ///< 记录时龙门架Y位置 (mm)
    float         gantryZ = 0.0f;            ///< 记录时龙门架Z位置 (mm)
    Eigen::Matrix4d rel_transform;           ///< 相对于标准Tag的变换 (标准基座系)
    std::string   snapshotPath;              ///< 快照路径 (可选)
    std::optional<TagDetection> tagData;     ///< Tag检测数据 (可选)

    PhotoPoint()
        : rel_transform(Eigen::Matrix4d::Identity())
    {
    }
};

/**
 * @brief 视觉伺服配方
 *
 * 包含示教阶段的完整数据: 标准点、拍照点列表、龙门架参考位置。
 */
struct DEV_CORRECTOR_API ServoRecipe
{
    std::string id;                    ///< 配方ID
    std::string name;                  ///< 配方名称
    double      createdTime = 0.0;     ///< 创建时间戳
    std::string description;           ///< 描述信息

    Pose6D        stdRobotPose;         ///< 标准点机械臂位姿 (基座坐标系)
    float         stdGantryX = 0.0f;    ///< 标准点龙门架X位置 (mm)
    float         stdGantryY = 0.0f;    ///< 标准点龙门架Y位置 (mm)
    float         stdGantryZ = 0.0f;    ///< 标准点龙门架Z位置 (mm)
    TagDetection  stdTagData;           ///< 标准点Tag检测数据
    Eigen::Matrix4d tBaseTagStd;        ///< 标准点Tag在基座系中的位姿 (4x4)

    std::vector<PhotoPoint> photoPoints;  ///< 拍照点位列表
    std::string             handEyeFile;  ///< 手眼标定文件路径

    ServoRecipe()
        : tBaseTagStd(Eigen::Matrix4d::Identity())
    {
    }
};

// ==================== 抽象接口定义 ====================

/**
 * @brief 纠偏算法抽象接口
 */
class DEV_CORRECTOR_API IDeviationCorrector
{
public:
    virtual ~IDeviationCorrector() = default;

    /**
     * @brief 设置手眼标定矩阵
     * @param tFlangeCam 法兰到相机的4x4齐次变换矩阵
     */
    virtual void setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam) = 0;

    /**
     * @brief 获取手眼标定矩阵
     * @return 法兰到相机的4x4齐次变换矩阵
     */
    virtual Eigen::Matrix4d getHandEyeCalibration() const = 0;

    /**
     * @brief 计算单点纠偏后的目标位姿
     * @param currentPose 当前机械臂位姿
     * @param deviation 视觉检测到的偏差 (相机坐标系, mm/deg)
     * @return 纠偏后的目标位姿
     */
    virtual Pose6D calculateCorrection(
        const Pose6D&        currentPose,
        const DeviationResult& deviation) = 0;

    /**
     * @brief 计算Tag在机械臂基座坐标系中的位姿
     * @param robotPose 机械臂当前位姿 (基座坐标系)
     * @param tagTvec Tag在相机系中的平移向量 (米)
     * @param tagRvec Tag在相机系中的旋转向量 (Rodrigues, 弧度)
     * @return Tag在基座系中的4x4齐次矩阵
     */
    virtual Eigen::Matrix4d computeTagInBase(
        const Pose6D&        robotPose,
        const Eigen::Vector3d& tagTvec,
        const Eigen::Vector3d& tagRvec) = 0;

    /**
     * @brief 偏差传播 — 计算所有拍照点的新位姿
     * @param tBaseTagNew 当前Tag在基座系中的位姿
     * @param relTransforms 各点位相对于标准Tag的变换矩阵
     * @return 所有拍照点在标准基座系下的新位姿
     */
    virtual std::vector<Pose6D> propagateDeviation(
        const Eigen::Matrix4d&            tBaseTagNew,
        const std::vector<Eigen::Matrix4d>& relTransforms) = 0;
};

// ==================== 具体实现类 ====================

/**
 * @brief 纠偏算法实现 (PIMPL模式)
 */
class DEV_CORRECTOR_API DeviationCorrector : public IDeviationCorrector
{
public:
    DeviationCorrector();
    ~DeviationCorrector() override;

    // --- 接口实现 ---
    void setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam) override;
    Eigen::Matrix4d getHandEyeCalibration() const override;

    Pose6D calculateCorrection(
        const Pose6D&        currentPose,
        const DeviationResult& deviation) override;

    Eigen::Matrix4d computeTagInBase(
        const Pose6D&        robotPose,
        const Eigen::Vector3d& tagTvec,
        const Eigen::Vector3d& tagRvec) override;

    std::vector<Pose6D> propagateDeviation(
        const Eigen::Matrix4d&            tBaseTagNew,
        const std::vector<Eigen::Matrix4d>& relTransforms) override;

    // --- 扩展功能 ---

    /**
     * @brief 从JSON文件加载手眼标定矩阵
     *
     * 支持两种格式:
     *   {"T": [[r00,r01,r02,r03], [r10,r11,r12,r13], [r20,r21,r22,r23], [r30,r31,r32,r33]]}
     *   {"result": [r00, r01, r02, ..., r33]}  (16个元素, 行优先)
     *
     * @param filepath JSON文件路径
     * @return 是否成功
     */
    bool loadHandEyeFromFile(const std::string& filepath);

    /**
     * @brief 位姿转4x4齐次矩阵 (Euler XYZ → 4x4)
     * @param pose 位姿 [x,y,z,rx,ry,rz] (mm, deg)
     * @param isDegree 角度是否为度 (默认true)
     * @return 4x4齐次矩阵 T = [R t; 0 1]
     */
    static Eigen::Matrix4d poseToMatrix(const Pose6D& pose, bool isDegree = true);

    /**
     * @brief 4x4齐次矩阵转位姿
     * @param matrix 4x4齐次矩阵
     * @param toDegree 是否输出度数 (默认true)
     * @return 位姿 [x,y,z,rx,ry,rz]
     */
    static Pose6D matrixToPose(const Eigen::Matrix4d& matrix, bool toDegree = true);

    /**
     * @brief Rodrigues旋转向量转旋转矩阵
     * @param rvec 旋转向量 (弧度, 模长为旋转角度)
     * @return 3x3旋转矩阵
     */
    static Eigen::Matrix3d rodriguesToMatrix(const Eigen::Vector3d& rvec);

    /**
     * @brief 旋转矩阵转欧拉角 (XYZ顺序, R = Rz*Ry*Rx)
     * @param r 3x3旋转矩阵
     * @param toDegree 是否输出度数
     * @return 欧拉角 [rx, ry, rz]
     */
    static Eigen::Vector3d matrixToEulerXyz(const Eigen::Matrix3d& r, bool toDegree = true);

    /**
     * @brief 欧拉角转旋转矩阵 (XYZ顺序)
     * @param euler 欧拉角 [rx, ry, rz]
     * @param isDegree 是否为度数
     * @return 3x3旋转矩阵 R = Rz*Ry*Rx
     */
    static Eigen::Matrix3d eulerXyzToMatrix(const Eigen::Vector3d& euler, bool isDegree = true);

private:
    struct Impl;
    Impl* pImpl_;
};

// ==================== 多点位视觉伺服控制器 ====================

/**
 * @brief 多点位视觉伺服控制器
 *
 * 管理完整的"示教→生产"工作流。
 *
 * 龙门架坐标系说明:
 *   龙门架承载机械臂基座, 龙门架移动时机械臂基座跟随移动。
 *   机械臂GetPosition()返回的位姿始终相对于当前基座位置。
 *   示教时记录每个拍照点的龙门架位置, 在计算相对变换时补偿
 *   基座偏移, 确保不同龙门架位置下的坐标变换正确。
 *
 *   生产阶段 computeNewPoses() 返回的位姿位于标准基座系
 *   (即标准点的龙门架位置处)。调用方需根据实际龙门架位置
 *   使用 adjustForGantry() 进行转换。
 */
class DEV_CORRECTOR_API MultiPointServo
{
public:
    MultiPointServo();
    explicit MultiPointServo(const std::string& handEyeFile);
    ~MultiPointServo();

    // ==================== 示教阶段 ====================

    /**
     * @brief 开始新的示教流程
     * @param name 配方名称 (为空则自动生成)
     * @return 新创建的空白配方
     */
    ServoRecipe startTeaching(const std::string& name = "");

    /**
     * @brief 记录标准点 (视觉基准点)
     * @param robotPose 机械臂当前位姿 (基座坐标系)
     * @param tagResult Tag检测结果
     * @param gantryX 当前龙门架X位置 (mm)
     * @param gantryY 当前龙门架Y位置 (mm)
     * @param gantryZ 当前龙门架Z位置 (mm)
     * @return 是否成功
     */
    bool recordStandardPoint(
        const Pose6D&       robotPose,
        const TagDetection& tagResult,
        float               gantryX,
        float               gantryY,
        float               gantryZ);

    /**
     * @brief 添加拍照点位
     * @param name 点位名称
     * @param robotPose 机械臂位姿 (当前基座坐标系)
     * @param gantryX 当前龙门架X位置 (mm)
     * @param gantryY 当前龙门架Y位置 (mm)
     * @param gantryZ 当前龙门架Z位置 (mm)
     * @return 当前拍照点总数
     */
    int addPhotoPoint(
        const std::string&  name,
        const Pose6D&       robotPose,
        float               gantryX,
        float               gantryY,
        float               gantryZ);

    /**
     * @brief 完成示教, 计算各点位相对于标准Tag的变换矩阵
     *
     * 在此阶段补偿龙门架基座偏移:
     *   T_std_flange_i = [I | G_i - G_std] @ T_base_i_flange_i
     *   rel_transform_i = inv(T_base_tag_std) @ T_std_flange_i
     *
     * @return 完成后的配方
     */
    ServoRecipe finishTeaching();

    // ==================== 生产阶段 ====================

    /**
     * @brief 计算所有拍照点的新位姿 (偏差传播)
     *
     * 返回的位姿位于标准基座系 (标准点龙门架位置处)。
     * 调用方需根据实际龙门架位置使用 adjustForGantry() 转换。
     *
     * @param robotPose 当前机械臂位姿 (基座坐标系, mm/deg)
     * @param tagResult 当前Tag检测结果
     * @param currentGantryX 生产时龙门架X位置 (mm)
     * @param currentGantryY 生产时龙门架Y位置 (mm)
     * @param currentGantryZ 生产时龙门架Z位置 (mm)
     * @return 拍照点新位姿列表 [(名称, 位姿在标准基座系), ...]
     */
    std::vector<std::pair<std::string, Pose6D>> computeNewPoses(
        const Pose6D&       robotPose,
        const TagDetection& tagResult,
        float               currentGantryX,
        float               currentGantryY,
        float               currentGantryZ);

    /**
     * @brief 将标准基座系下的位姿转换到目标龙门架位置下的基座系
     *
     * 龙门架只有平移无旋转, 因此仅调整位姿的X/Y/Z分量。
     *
     * @param poseStdFrame 标准基座系下的位姿
     * @param stdGantryX 标准点龙门架X (mm)
     * @param stdGantryY 标准点龙门架Y (mm)
     * @param stdGantryZ 标准点龙门架Z (mm)
     * @param targetGantryX 目标龙门架X (mm)
     * @param targetGantryY 目标龙门架Y (mm)
     * @param targetGantryZ 目标龙门架Z (mm)
     * @return 目标基座系下的位姿
     */
    static Pose6D adjustForGantry(
        const Pose6D& poseStdFrame,
        float         stdGantryX,
        float         stdGantryY,
        float         stdGantryZ,
        float         targetGantryX,
        float         targetGantryY,
        float         targetGantryZ);

    // ==================== 配方持久化 ====================

    /**
     * @brief 从JSON文件加载配方
     * @param filepath 配方文件路径
     * @return 是否成功
     */
    bool loadRecipe(const std::string& filepath);

    /**
     * @brief 保存配方到JSON文件
     * @param filepath 保存路径
     * @return 是否成功
     */
    bool saveRecipe(const std::string& filepath) const;

    // ==================== 手眼标定 ====================

    /**
     * @brief 设置手眼标定矩阵
     * @param tFlangeCam 法兰到相机的4x4齐次变换矩阵
     */
    void setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam);

    /**
     * @brief 从JSON文件加载手眼标定矩阵
     * @param filepath JSON文件路径
     * @return 是否成功
     */
    bool loadHandEyeFromFile(const std::string& filepath);

    // ==================== 访问器 ====================

    /**
     * @brief 获取当前配方 (只读)
     */
    const ServoRecipe& getCurrentRecipe() const;

private:
    struct Impl;
    Impl* pImpl_;
};

} // namespace vision_servo
