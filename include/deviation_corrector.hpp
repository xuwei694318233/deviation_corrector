#pragma once

/**
 * @file deviation_corrector.hpp
 * @brief 视觉伺服纠偏算法库 - 标准化接口 (动态库版本)
 *
 * 提供工业机器人视觉伺服纠偏的核心算法实现
 * 支持单点纠偏和多点位偏差传播
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

    // ==================== 数据结构定义 ====================

    /**
     * @brief 6自由度位姿
     */
    struct DEV_CORRECTOR_API Pose6D
    {
        double x = 0.0;  ///< X位置 (mm)
        double y = 0.0;  ///< Y位置 (mm)
        double z = 0.0;  ///< Z位置 (mm)
        double rx = 0.0; ///< 绕X轴旋转角度 (deg)
        double ry = 0.0; ///< 绕Y轴旋转角度 (deg)
        double rz = 0.0; ///< 绕Z轴旋转角度 (deg)

        Pose6D() = default;
        Pose6D(double x_, double y_, double z_, double rx_, double ry_, double rz_)
            : x(x_), y(y_), z(z_), rx(rx_), ry(ry_), rz(rz_) {}

        /// 转换为向量 [x, y, z, rx, ry, rz]
        std::vector<double> toVector() const
        {
            return {x, y, z, rx, ry, rz};
        }

        /// 从向量创建
        static Pose6D fromVector(const std::vector<double> &v)
        {
            if (v.size() >= 6)
            {
                return Pose6D(v[0], v[1], v[2], v[3], v[4], v[5]);
            }
            return Pose6D();
        }
    };

    /**
     * @brief 偏差结果
     */
    struct DEV_CORRECTOR_API DeviationResult
    {
        double dx = 0.0;  ///< X方向偏差 (mm)
        double dy = 0.0;  ///< Y方向偏差 (mm)
        double dz = 0.0;  ///< Z方向偏差 (mm)
        double drx = 0.0; ///< 绕X轴旋转偏差 (deg)
        double dry = 0.0; ///< 绕Y轴旋转偏差 (deg)
        double drz = 0.0; ///< 绕Z轴旋转偏差 (deg)

        DeviationResult() = default;
        DeviationResult(double dx_, double dy_, double dz_, double drx_, double dry_, double drz_)
            : dx(dx_), dy(dy_), dz(dz_), drx(drx_), dry(dry_), drz(drz_) {}

        /// 创建仅平移的偏差 (3DOF)
        static DeviationResult translation(double dx_, double dy_, double dz_)
        {
            return DeviationResult(dx_, dy_, dz_, 0.0, 0.0, 0.0);
        }

        /// 创建XY平面纠偏 (常用场景)
        static DeviationResult xyPlane(double dx_, double dy_, double drz_)
        {
            return DeviationResult(dx_, dy_, 0.0, 0.0, 0.0, drz_);
        }
    };

    /**
     * @brief Tag检测结果
     */
    struct DEV_CORRECTOR_API TagDetection
    {
        int id = 0;             ///< Tag ID
        Eigen::Vector3d tvec;   ///< 平移向量 (相机坐标系, 米)
        Eigen::Vector3d rvec;   ///< 旋转向量 (Rodrigues, 弧度)
        Eigen::Vector3d euler;  ///< 欧拉角 XYZ (度)
        Eigen::Vector2d center; ///< 像素中心坐标

        TagDetection() : tvec(Eigen::Vector3d::Zero()),
                         rvec(Eigen::Vector3d::Zero()),
                         euler(Eigen::Vector3d::Zero()),
                         center(Eigen::Vector2d::Zero()) {}
    };

    /**
     * @brief 拍照点位数据
     */
    struct DEV_CORRECTOR_API PhotoPoint
    {
        std::string name;                     ///< 点位名称
        Pose6D pose;                          ///< 机械臂位姿
        Eigen::Matrix4d rel_transform;        ///< 相对于标准Tag的变换矩阵
        std::string snapshot_path;            ///< 快照路径 (可选)
        std::optional<TagDetection> tag_data; ///< Tag检测数据 (可选)

        PhotoPoint() : rel_transform(Eigen::Matrix4d::Identity()) {}
    };

    /**
     * @brief 视觉伺服配方
     */
    struct DEV_CORRECTOR_API ServoRecipe
    {
        std::string id;            ///< 配方ID
        std::string name;          ///< 配方名称
        double created_time = 0.0; ///< 创建时间戳
        std::string description;   ///< 描述

        Pose6D std_robot_pose;          ///< 标准位置机械臂位姿
        TagDetection std_tag_data;      ///< 标准位置Tag检测数据
        Eigen::Matrix4d T_base_tag_std; ///< Tag在基座系中的位姿矩阵

        std::vector<PhotoPoint> photo_points; ///< 拍照点位列表
        std::string hand_eye_file;            ///< 手眼标定文件路径

        ServoRecipe() : T_base_tag_std(Eigen::Matrix4d::Identity()) {}
    };

    // ==================== 抽象接口定义 ====================

    /**
     * @brief 纠偏算法接口 (抽象基类)
     */
    class DEV_CORRECTOR_API IDeviationCorrector
    {
    public:
        virtual ~IDeviationCorrector() = default;

        /**
         * @brief 设置手眼标定矩阵
         * @param T_flange_cam 法兰到相机的变换矩阵 (4x4)
         */
        virtual void setHandEyeCalibration(const Eigen::Matrix4d &T_flange_cam) = 0;

        /**
         * @brief 获取手眼标定矩阵
         */
        virtual Eigen::Matrix4d getHandEyeCalibration() const = 0;

        /**
         * @brief 计算纠偏后的目标位姿
         * @param current_pose 当前机械臂位姿
         * @param deviation 视觉检测到的偏差
         * @return 纠偏后的目标位姿
         */
        virtual Pose6D calculateCorrection(
            const Pose6D &current_pose,
            const DeviationResult &deviation) = 0;

        /**
         * @brief 计算Tag在基座坐标系中的位姿
         * @param robot_pose 机械臂当前位姿
         * @param tag_tvec Tag在相机系中的平移向量 (米)
         * @param tag_rvec Tag在相机系中的旋转向量 (Rodrigues, 弧度)
         * @return Tag在基座系中的4x4齐次矩阵
         */
        virtual Eigen::Matrix4d computeTagInBase(
            const Pose6D &robot_pose,
            const Eigen::Vector3d &tag_tvec,
            const Eigen::Vector3d &tag_rvec) = 0;

        /**
         * @brief 偏差传播 - 计算所有点位的新位姿
         * @param T_base_tag_new 新的Tag在基座系中的位姿
         * @param rel_transforms 各点位相对于Tag的变换矩阵列表
         * @return 所有拍照点的新位姿列表
         */
        virtual std::vector<Pose6D> propagateDeviation(
            const Eigen::Matrix4d &T_base_tag_new,
            const std::vector<Eigen::Matrix4d> &rel_transforms) = 0;
    };

    // ==================== 具体实现类 ====================

    /**
     * @brief 纠偏算法实现类
     */
    class DEV_CORRECTOR_API DeviationCorrector : public IDeviationCorrector
    {
    public:
        DeviationCorrector();
        ~DeviationCorrector() override;

        // 实现接口方法
        void setHandEyeCalibration(const Eigen::Matrix4d &T_flange_cam) override;
        Eigen::Matrix4d getHandEyeCalibration() const override;

        Pose6D calculateCorrection(
            const Pose6D &current_pose,
            const DeviationResult &deviation) override;

        Eigen::Matrix4d computeTagInBase(
            const Pose6D &robot_pose,
            const Eigen::Vector3d &tag_tvec,
            const Eigen::Vector3d &tag_rvec) override;

        std::vector<Pose6D> propagateDeviation(
            const Eigen::Matrix4d &T_base_tag_new,
            const std::vector<Eigen::Matrix4d> &rel_transforms) override;

        // ==================== 扩展功能 ====================

        /**
         * @brief 从文件加载手眼标定矩阵 (JSON格式)
         * @param filepath JSON文件路径
         * @return 是否成功
         */
        bool loadHandEyeFromFile(const std::string &filepath);

        /**
         * @brief 位姿向量转齐次矩阵 (Euler XYZ)
         * @param pose 位姿 [x, y, z, rx, ry, rz]
         * @param is_degree 角度是否为度数 (默认true)
         * @return 4x4齐次矩阵
         */
        static Eigen::Matrix4d poseToMatrix(const Pose6D &pose, bool is_degree = true);

        /**
         * @brief 齐次矩阵转位姿向量 (Euler XYZ)
         * @param matrix 4x4齐次矩阵
         * @param to_degree 是否输出度数 (默认true)
         * @return 位姿 [x, y, z, rx, ry, rz]
         */
        static Pose6D matrixToPose(const Eigen::Matrix4d &matrix, bool to_degree = true);

        /**
         * @brief 旋转向量转旋转矩阵 (Rodrigues)
         * @param rvec 旋转向量 (弧度)
         * @return 3x3旋转矩阵
         */
        static Eigen::Matrix3d rodriguesToMatrix(const Eigen::Vector3d &rvec);

        /**
         * @brief 旋转矩阵转欧拉角 (XYZ顺序)
         * @param R 旋转矩阵
         * @param to_degree 是否输出度数
         * @return 欧拉角 [rx, ry, rz]
         */
        static Eigen::Vector3d matrixToEulerXYZ(const Eigen::Matrix3d &R, bool to_degree = true);

        /**
         * @brief 欧拉角转旋转矩阵 (XYZ顺序)
         * @param euler 欧拉角 [rx, ry, rz]
         * @param is_degree 是否为度数
         * @return 3x3旋转矩阵
         */
        static Eigen::Matrix3d eulerXYZToMatrix(const Eigen::Vector3d &euler, bool is_degree = true);

    private:
        struct Impl;
        Impl *pImpl_; // PIMPL 模式，隐藏实现细节
    };

    // ==================== 多点位视觉伺服控制器 ====================

    /**
     * @brief 多点位视觉伺服控制器
     */
    class DEV_CORRECTOR_API MultiPointServo
    {
    public:
        MultiPointServo();
        explicit MultiPointServo(const std::string &hand_eye_file);
        ~MultiPointServo();

        /**
         * @brief 开始新的示教流程
         * @param name 配方名称
         * @return 新创建的配方
         */
        ServoRecipe startTeaching(const std::string &name = "");

        /**
         * @brief 记录标准点 (偏差计算点位)
         * @param robot_pose 机械臂当前位姿
         * @param tag_result Tag检测结果
         * @return 是否成功
         */
        bool recordStandardPoint(const Pose6D &robot_pose, const TagDetection &tag_result);

        /**
         * @brief 添加拍照点位
         * @param name 点位名称
         * @param robot_pose 机械臂位姿
         * @return 当前点位总数
         */
        int addPhotoPoint(const std::string &name, const Pose6D &robot_pose);

        /**
         * @brief 完成示教，计算相对变换
         * @return 完成后的配方
         */
        ServoRecipe finishTeaching();

        /**
         * @brief 生产阶段 - 计算所有点位新位姿
         * @param robot_pose 当前机械臂位姿
         * @param tag_result Tag检测结果
         * @return 新位姿列表 [(点位名, 新位姿), ...]
         */
        std::vector<std::pair<std::string, Pose6D>> computeNewPoses(
            const Pose6D &robot_pose,
            const TagDetection &tag_result);

        /**
         * @brief 加载配方
         * @param filepath 配方文件路径
         * @return 是否成功
         */
        bool loadRecipe(const std::string &filepath);

        /**
         * @brief 保存配方
         * @param filepath 保存路径
         * @return 是否成功
         */
        bool saveRecipe(const std::string &filepath);

        /**
         * @brief 获取当前配方
         */
        const ServoRecipe &getCurrentRecipe() const;

        /**
         * @brief 获取纠偏器
         */
        DeviationCorrector &getCorrector();

    private:
        struct Impl;
        Impl *pImpl_; // PIMPL 模式
    };

    // ==================== C 风格接口 (便于其他语言调用) ====================

    extern "C"
    {

        /**
         * @brief 创建纠偏器实例
         * @return 纠偏器指针
         */
        DEV_CORRECTOR_API DeviationCorrector *deviation_corrector_create();

        /**
         * @brief 销毁纠偏器实例
         * @param corrector 纠偏器指针
         */
        DEV_CORRECTOR_API void deviation_corrector_destroy(DeviationCorrector *corrector);

        /**
         * @brief 设置手眼标定矩阵
         * @param corrector 纠偏器指针
         * @param matrix 4x4矩阵数据 (16个double, 行优先)
         */
        DEV_CORRECTOR_API void deviation_corrector_set_hand_eye(
            DeviationCorrector *corrector,
            const double *matrix);

        /**
         * @brief 计算纠偏 (C接口)
         * @param corrector 纠偏器指针
         * @param current_pose 当前位姿 [x, y, z, rx, ry, rz]
         * @param deviation 偏差 [dx, dy, dz, drx, dry, drz]
         * @param out_pose 输出位姿 [x, y, z, rx, ry, rz]
         */
        DEV_CORRECTOR_API void deviation_corrector_calculate(
            DeviationCorrector *corrector,
            const double *current_pose,
            const double *deviation,
            double *out_pose);

        /**
         * @brief 获取库版本号
         * @return 版本字符串
         */
        DEV_CORRECTOR_API const char *deviation_corrector_version();

    } // extern "C"

} // namespace vision_servo
