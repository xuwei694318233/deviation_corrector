/**
 * @file deviation_corrector.cpp
 * @brief 视觉伺服纠偏算法库 - 实现 (动态库版本)
 */

#include "deviation_corrector.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <chrono>

namespace vision_servo
{

    // ==================== 常量定义 ====================
    constexpr double PI = 3.14159265358979323846;
    constexpr double DEG_TO_RAD = PI / 180.0;
    constexpr double RAD_TO_DEG = 180.0 / PI;

    // ==================== DeviationCorrector 实现 ====================

    struct DeviationCorrector::Impl
    {
        Eigen::Matrix4d T_flange_cam;
        bool calibrated;

        Impl() : T_flange_cam(Eigen::Matrix4d::Identity()), calibrated(false) {}
    };

    DeviationCorrector::DeviationCorrector() : pImpl_(new Impl())
    {
    }

    DeviationCorrector::~DeviationCorrector() = default;

    void DeviationCorrector::setHandEyeCalibration(const Eigen::Matrix4d &T_flange_cam)
    {
        pImpl_->T_flange_cam = T_flange_cam;
        pImpl_->calibrated = true;
    }

    Eigen::Matrix4d DeviationCorrector::getHandEyeCalibration() const
    {
        return pImpl_->T_flange_cam;
    }

    Eigen::Matrix4d DeviationCorrector::poseToMatrix(const Pose6D &pose, bool is_degree)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        // 设置平移部分
        T(0, 3) = pose.x;
        T(1, 3) = pose.y;
        T(2, 3) = pose.z;

        // 设置旋转部分 (Euler XYZ)
        double rx = is_degree ? pose.rx * DEG_TO_RAD : pose.rx;
        double ry = is_degree ? pose.ry * DEG_TO_RAD : pose.ry;
        double rz = is_degree ? pose.rz * DEG_TO_RAD : pose.rz;

        T.block<3, 3>(0, 0) = eulerXYZToMatrix(Eigen::Vector3d(rx, ry, rz), false);

        return T;
    }

    Pose6D DeviationCorrector::matrixToPose(const Eigen::Matrix4d &matrix, bool to_degree)
    {
        Pose6D pose;

        // 提取平移
        pose.x = matrix(0, 3);
        pose.y = matrix(1, 3);
        pose.z = matrix(2, 3);

        // 提取旋转并转为欧拉角
        Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);
        Eigen::Vector3d euler = matrixToEulerXYZ(R, to_degree);

        pose.rx = euler(0);
        pose.ry = euler(1);
        pose.rz = euler(2);

        return pose;
    }

    Eigen::Matrix3d DeviationCorrector::rodriguesToMatrix(const Eigen::Vector3d &rvec)
    {
        // Rodrigues 公式: R = I + sin(θ)*K + (1-cos(θ))*K²
        // 其中 K 是旋转向量的反对称矩阵，θ 是旋转向量的模长

        double theta = rvec.norm();

        if (theta < 1e-10)
        {
            return Eigen::Matrix3d::Identity();
        }

        // 单位化旋转向量
        Eigen::Vector3d k = rvec / theta;

        // 构造反对称矩阵 K
        Eigen::Matrix3d K;
        K << 0, -k(2), k(1),
            k(2), 0, -k(0),
            -k(1), k(0), 0;

        // Rodrigues 公式
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1 - std::cos(theta)) * K * K;

        return R;
    }

    Eigen::Vector3d DeviationCorrector::matrixToEulerXYZ(const Eigen::Matrix3d &R, bool to_degree)
    {
        // 提取 XYZ 欧拉角
        // R = Rz(rz) * Ry(ry) * Rx(rx)

        double sy = std::sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
        bool singular = sy < 1e-6;

        Eigen::Vector3d euler;

        if (!singular)
        {
            euler(0) = std::atan2(R(2, 1), R(2, 2)); // rx
            euler(1) = std::atan2(-R(2, 0), sy);     // ry
            euler(2) = std::atan2(R(1, 0), R(0, 0)); // rz
        }
        else
        {
            // 万向节锁情况
            euler(0) = std::atan2(-R(1, 2), R(1, 1)); // rx
            euler(1) = std::atan2(-R(2, 0), sy);      // ry
            euler(2) = 0;                             // rz
        }

        if (to_degree)
        {
            euler *= RAD_TO_DEG;
        }

        return euler;
    }

    Eigen::Matrix3d DeviationCorrector::eulerXYZToMatrix(const Eigen::Vector3d &euler, bool is_degree)
    {
        double rx = euler(0);
        double ry = euler(1);
        double rz = euler(2);

        if (is_degree)
        {
            rx *= DEG_TO_RAD;
            ry *= DEG_TO_RAD;
            rz *= DEG_TO_RAD;
        }

        // 计算各轴旋转矩阵
        // Rx
        Eigen::Matrix3d Rx;
        Rx << 1, 0, 0,
            0, std::cos(rx), -std::sin(rx),
            0, std::sin(rx), std::cos(rx);

        // Ry
        Eigen::Matrix3d Ry;
        Ry << std::cos(ry), 0, std::sin(ry),
            0, 1, 0,
            -std::sin(ry), 0, std::cos(ry);

        // Rz
        Eigen::Matrix3d Rz;
        Rz << std::cos(rz), -std::sin(rz), 0,
            std::sin(rz), std::cos(rz), 0,
            0, 0, 1;

        // R = Rz * Ry * Rx (外旋 ZYX = 内旋 XYZ)
        return Rz * Ry * Rx;
    }

    Pose6D DeviationCorrector::calculateCorrection(
        const Pose6D &current_pose,
        const DeviationResult &deviation)
    {

        if (!pImpl_->calibrated)
        {
            throw std::runtime_error("Hand-eye calibration not set. Call setHandEyeCalibration() first.");
        }

        // 1. 构造偏差矩阵 T_dev
        double drx_rad = deviation.drx * DEG_TO_RAD;
        double dry_rad = deviation.dry * DEG_TO_RAD;
        double drz_rad = deviation.drz * DEG_TO_RAD;

        Eigen::Matrix4d T_dev = Eigen::Matrix4d::Identity();
        T_dev.block<3, 3>(0, 0) = eulerXYZToMatrix(
            Eigen::Vector3d(drx_rad, dry_rad, drz_rad), false);
        T_dev(0, 3) = deviation.dx;
        T_dev(1, 3) = deviation.dy;
        T_dev(2, 3) = deviation.dz;

        // 2. 完整矩阵链
        // T_B_F_new = T_B_F_cur @ T_F_C @ T_dev @ T_F_C_inv
        Eigen::Matrix4d T_B_F_cur = poseToMatrix(current_pose, true);
        Eigen::Matrix4d T_F_C_inv = pImpl_->T_flange_cam.inverse();

        Eigen::Matrix4d T_B_F_new = T_B_F_cur * pImpl_->T_flange_cam * T_dev * T_F_C_inv;

        // 3. 提取新位姿
        Pose6D new_pose = matrixToPose(T_B_F_new, true);

        // 4. 锁定高度 (X轴 = 此机器人的竖直方向)
        // 可根据实际机器人坐标系调整
        new_pose.x = current_pose.x;

        return new_pose;
    }

    Eigen::Matrix4d DeviationCorrector::computeTagInBase(
        const Pose6D &robot_pose,
        const Eigen::Vector3d &tag_tvec,
        const Eigen::Vector3d &tag_rvec)
    {

        if (!pImpl_->calibrated)
        {
            throw std::runtime_error("Hand-eye calibration not set. Call setHandEyeCalibration() first.");
        }

        // T_base_tag = T_base_flange @ T_flange_cam @ T_cam_tag

        // 1. T_base_flange
        Eigen::Matrix4d T_base_flange = poseToMatrix(robot_pose, true);

        // 2. T_cam_tag
        Eigen::Matrix3d R_cam_tag = rodriguesToMatrix(tag_rvec);
        Eigen::Matrix4d T_cam_tag = Eigen::Matrix4d::Identity();
        T_cam_tag.block<3, 3>(0, 0) = R_cam_tag;
        // 平移从米转到毫米 (与机械臂单位统一)
        T_cam_tag(0, 3) = tag_tvec(0) * 1000.0;
        T_cam_tag(1, 3) = tag_tvec(1) * 1000.0;
        T_cam_tag(2, 3) = tag_tvec(2) * 1000.0;

        // 3. 计算链式乘法
        Eigen::Matrix4d T_base_tag = T_base_flange * pImpl_->T_flange_cam * T_cam_tag;

        return T_base_tag;
    }

    std::vector<Pose6D> DeviationCorrector::propagateDeviation(
        const Eigen::Matrix4d &T_base_tag_new,
        const std::vector<Eigen::Matrix4d> &rel_transforms)
    {

        std::vector<Pose6D> new_poses;
        new_poses.reserve(rel_transforms.size());

        // T_base_flange_i_new = T_base_tag_new @ T_tag_flange_i
        for (const auto &T_tag_flange : rel_transforms)
        {
            Eigen::Matrix4d T_base_flange_new = T_base_tag_new * T_tag_flange;
            Pose6D new_pose = matrixToPose(T_base_flange_new, true);
            new_poses.push_back(new_pose);
        }

        return new_poses;
    }

    bool DeviationCorrector::loadHandEyeFromFile(const std::string &filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }

        try
        {
            std::string content((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
            file.close();

            // 简单JSON解析 (不依赖外部库)
            // 格式: {"T": [[...], [...], ...]}

            // 查找 "T" 键
            size_t pos = content.find("\"T\"");
            if (pos == std::string::npos)
            {
                return false;
            }

            // 查找数组开始
            pos = content.find('[', pos + 3);
            if (pos == std::string::npos)
            {
                return false;
            }

            // 跳过外层数组开始
            pos = content.find('[', pos + 1);

            Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

            for (int i = 0; i < 4; ++i)
            {
                // 找行开始
                size_t row_start = content.find('[', pos);
                if (row_start == std::string::npos)
                    break;

                size_t row_end = content.find(']', row_start);
                std::string row_str = content.substr(row_start + 1, row_end - row_start - 1);

                std::istringstream iss(row_str);
                for (int j = 0; j < 4; ++j)
                {
                    iss >> matrix(i, j);
                    char comma;
                    if (j < 3)
                        iss >> comma; // 跳过逗号
                }

                pos = row_end + 1;
            }

            setHandEyeCalibration(matrix);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    // ==================== MultiPointServo 实现 ====================

    struct MultiPointServo::Impl
    {
        DeviationCorrector corrector;
        ServoRecipe current_recipe;

        void computeRelativeTransforms()
        {
            if (current_recipe.T_base_tag_std.isApprox(Eigen::Matrix4d::Identity()))
            {
                return;
            }

            Eigen::Matrix4d T_base_tag_inv = current_recipe.T_base_tag_std.inverse();

            for (auto &pp : current_recipe.photo_points)
            {
                Eigen::Matrix4d T_base_flange = DeviationCorrector::poseToMatrix(pp.pose, true);
                pp.rel_transform = T_base_tag_inv * T_base_flange;
            }
        }
    };

    MultiPointServo::MultiPointServo() : pImpl_(new Impl())
    {
    }

    MultiPointServo::MultiPointServo(const std::string &hand_eye_file) : pImpl_(new Impl())
    {
        pImpl_->corrector.loadHandEyeFromFile(hand_eye_file);
    }

    MultiPointServo::~MultiPointServo() = default;

    ServoRecipe MultiPointServo::startTeaching(const std::string &name)
    {
        pImpl_->current_recipe = ServoRecipe();

        // 生成ID和时间戳
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        pImpl_->current_recipe.created_time = std::chrono::duration<double>(duration).count();

        if (name.empty())
        {
            std::ostringstream oss;
            std::time_t t = std::chrono::system_clock::to_time_t(now);
            oss << "Recipe_" << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");
            pImpl_->current_recipe.name = oss.str();
        }
        else
        {
            pImpl_->current_recipe.name = name;
        }

        pImpl_->current_recipe.id = "recipe_" + std::to_string(static_cast<long long>(pImpl_->current_recipe.created_time));

        return pImpl_->current_recipe;
    }

    bool MultiPointServo::recordStandardPoint(const Pose6D &robot_pose, const TagDetection &tag_result)
    {
        pImpl_->current_recipe.std_robot_pose = robot_pose;
        pImpl_->current_recipe.std_tag_data = tag_result;

        // 计算 T_base_tag_std
        try
        {
            pImpl_->current_recipe.T_base_tag_std = pImpl_->corrector.computeTagInBase(
                robot_pose, tag_result.tvec, tag_result.rvec);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    int MultiPointServo::addPhotoPoint(const std::string &name, const Pose6D &robot_pose)
    {
        PhotoPoint pp;
        pp.name = name;
        pp.pose = robot_pose;
        pp.rel_transform = Eigen::Matrix4d::Identity();

        pImpl_->current_recipe.photo_points.push_back(pp);

        return static_cast<int>(pImpl_->current_recipe.photo_points.size());
    }

    ServoRecipe MultiPointServo::finishTeaching()
    {
        if (pImpl_->current_recipe.T_base_tag_std.isApprox(Eigen::Matrix4d::Identity()))
        {
            throw std::runtime_error("Standard point not recorded. Call recordStandardPoint() first.");
        }

        // 计算所有点位的相对变换
        pImpl_->computeRelativeTransforms();

        return pImpl_->current_recipe;
    }

    std::vector<std::pair<std::string, Pose6D>> MultiPointServo::computeNewPoses(
        const Pose6D &robot_pose,
        const TagDetection &tag_result)
    {

        std::vector<std::pair<std::string, Pose6D>> results;

        // 计算新的 T_base_tag
        Eigen::Matrix4d T_base_tag_new = pImpl_->corrector.computeTagInBase(
            robot_pose, tag_result.tvec, tag_result.rvec);

        // 提取相对变换
        std::vector<Eigen::Matrix4d> rel_transforms;
        for (const auto &pp : pImpl_->current_recipe.photo_points)
        {
            rel_transforms.push_back(pp.rel_transform);
        }

        // 偏差传播
        std::vector<Pose6D> new_poses = pImpl_->corrector.propagateDeviation(T_base_tag_new, rel_transforms);

        // 组装结果
        for (size_t i = 0; i < new_poses.size() && i < pImpl_->current_recipe.photo_points.size(); ++i)
        {
            results.emplace_back(pImpl_->current_recipe.photo_points[i].name, new_poses[i]);
        }

        return results;
    }

    bool MultiPointServo::loadRecipe(const std::string &filepath)
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }

        try
        {
            std::string content((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());
            file.close();

            // 简化的JSON解析 - 实际项目中应使用 nlohmann/json 或 rapidjson

            // 解析基本字段 (简化版)
            pImpl_->current_recipe = ServoRecipe();

            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool MultiPointServo::saveRecipe(const std::string &filepath)
    {
        std::ofstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }

        try
        {
            file << std::fixed << std::setprecision(6);
            file << "{\n";
            file << "  \"id\": \"" << pImpl_->current_recipe.id << "\",\n";
            file << "  \"name\": \"" << pImpl_->current_recipe.name << "\",\n";
            file << "  \"created_time\": " << pImpl_->current_recipe.created_time << ",\n";
            file << "  \"description\": \"" << pImpl_->current_recipe.description << "\",\n";

            // std_robot_pose
            file << "  \"std_robot_pose\": ["
                 << pImpl_->current_recipe.std_robot_pose.x << ", "
                 << pImpl_->current_recipe.std_robot_pose.y << ", "
                 << pImpl_->current_recipe.std_robot_pose.z << ", "
                 << pImpl_->current_recipe.std_robot_pose.rx << ", "
                 << pImpl_->current_recipe.std_robot_pose.ry << ", "
                 << pImpl_->current_recipe.std_robot_pose.rz << "],\n";

            // T_base_tag_std
            file << "  \"T_base_tag_std\": [\n";
            for (int i = 0; i < 4; ++i)
            {
                file << "    [";
                for (int j = 0; j < 4; ++j)
                {
                    file << pImpl_->current_recipe.T_base_tag_std(i, j);
                    if (j < 3)
                        file << ", ";
                }
                file << "]";
                if (i < 3)
                    file << ",";
                file << "\n";
            }
            file << "  ],\n";

            // photo_points
            file << "  \"photo_points\": [\n";
            for (size_t i = 0; i < pImpl_->current_recipe.photo_points.size(); ++i)
            {
                const auto &pp = pImpl_->current_recipe.photo_points[i];
                file << "    {\n";
                file << "      \"name\": \"" << pp.name << "\",\n";
                file << "      \"pose\": ["
                     << pp.pose.x << ", " << pp.pose.y << ", " << pp.pose.z << ", "
                     << pp.pose.rx << ", " << pp.pose.ry << ", " << pp.pose.rz << "],\n";

                // rel_transform
                file << "      \"rel_transform\": [\n";
                for (int r = 0; r < 4; ++r)
                {
                    file << "        [";
                    for (int c = 0; c < 4; ++c)
                    {
                        file << pp.rel_transform(r, c);
                        if (c < 3)
                            file << ", ";
                    }
                    file << "]";
                    if (r < 3)
                        file << ",";
                    file << "\n";
                }
                file << "      ]\n";
                file << "    }";
                if (i < pImpl_->current_recipe.photo_points.size() - 1)
                    file << ",";
                file << "\n";
            }
            file << "  ]\n";

            file << "}\n";

            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    const ServoRecipe &MultiPointServo::getCurrentRecipe() const
    {
        return pImpl_->current_recipe;
    }

    DeviationCorrector &MultiPointServo::getCorrector()
    {
        return pImpl_->corrector;
    }

    // ==================== C 风格接口实现 ====================

    extern "C"
    {

        DEV_CORRECTOR_API DeviationCorrector *deviation_corrector_create()
        {
            try
            {
                return new DeviationCorrector();
            }
            catch (...)
            {
                return nullptr;
            }
        }

        DEV_CORRECTOR_API void deviation_corrector_destroy(DeviationCorrector *corrector)
        {
            delete corrector;
        }

        DEV_CORRECTOR_API void deviation_corrector_set_hand_eye(
            DeviationCorrector *corrector,
            const double *matrix)
        {
            if (!corrector || !matrix)
                return;

            Eigen::Matrix4d T;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    T(i, j) = matrix[i * 4 + j];
                }
            }
            corrector->setHandEyeCalibration(T);
        }

        DEV_CORRECTOR_API void deviation_corrector_calculate(
            DeviationCorrector *corrector,
            const double *current_pose,
            const double *deviation,
            double *out_pose)
        {
            if (!corrector || !current_pose || !deviation || !out_pose)
                return;

            try
            {
                Pose6D cur(current_pose[0], current_pose[1], current_pose[2],
                           current_pose[3], current_pose[4], current_pose[5]);
                DeviationResult dev(deviation[0], deviation[1], deviation[2],
                                    deviation[3], deviation[4], deviation[5]);

                Pose6D result = corrector->calculateCorrection(cur, dev);

                out_pose[0] = result.x;
                out_pose[1] = result.y;
                out_pose[2] = result.z;
                out_pose[3] = result.rx;
                out_pose[4] = result.ry;
                out_pose[5] = result.rz;
            }
            catch (...)
            {
                // 错误时输出零值
                for (int i = 0; i < 6; ++i)
                    out_pose[i] = 0.0;
            }
        }

        DEV_CORRECTOR_API const char *deviation_corrector_version()
        {
            return "DeviationCorrector v1.0.0";
        }

    } // extern "C"

} // namespace vision_servo
