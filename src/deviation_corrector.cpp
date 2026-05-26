/**
 * @file deviation_corrector.cpp
 * @brief 视觉伺服纠偏算法库 - 实现 (动态库版本)
 */

#include "deviation_corrector.hpp"
#include "../thirdparty/json/json.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <ctime>

namespace vision_servo
{

// ==================== 内部常量 ====================

constexpr double kPi      = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEpsilon  = 1e-10;

// ==================== DeviationCorrector 实现 ====================

struct DeviationCorrector::Impl
{
    Eigen::Matrix4d tFlangeCam;
    bool            calibrated;

    Impl()
        : tFlangeCam(Eigen::Matrix4d::Identity())
        , calibrated(false)
    {
    }
};

DeviationCorrector::DeviationCorrector()
    : pImpl_(new Impl())
{
}

DeviationCorrector::~DeviationCorrector() = default;

void DeviationCorrector::setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam)
{
    pImpl_->tFlangeCam  = tFlangeCam;
    pImpl_->calibrated  = true;
}

Eigen::Matrix4d DeviationCorrector::getHandEyeCalibration() const
{
    return pImpl_->tFlangeCam;
}

Eigen::Matrix4d DeviationCorrector::poseToMatrix(const Pose6D& pose, bool isDegree)
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity();

    t(0, 3) = pose.x;
    t(1, 3) = pose.y;
    t(2, 3) = pose.z;

    double rx = isDegree ? pose.rx * kDegToRad : pose.rx;
    double ry = isDegree ? pose.ry * kDegToRad : pose.ry;
    double rz = isDegree ? pose.rz * kDegToRad : pose.rz;

    t.block<3, 3>(0, 0) = eulerXyzToMatrix(Eigen::Vector3d(rx, ry, rz), false);

    return t;
}

Pose6D DeviationCorrector::matrixToPose(const Eigen::Matrix4d& matrix, bool toDegree)
{
    Pose6D pose;
    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);

    Eigen::Matrix3d r = matrix.block<3, 3>(0, 0);
    Eigen::Vector3d euler = matrixToEulerXyz(r, toDegree);
    pose.rx = euler(0);
    pose.ry = euler(1);
    pose.rz = euler(2);

    return pose;
}

Eigen::Matrix3d DeviationCorrector::rodriguesToMatrix(const Eigen::Vector3d& rvec)
{
    double theta = rvec.norm();

    if (theta < kEpsilon)
    {
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d k = rvec / theta;

    Eigen::Matrix3d kMat;
    kMat << 0,      -k(2),   k(1),
            k(2),    0,     -k(0),
            -k(1),    k(0),   0;

    return Eigen::Matrix3d::Identity()
            + std::sin(theta) * kMat
            + (1.0 - std::cos(theta)) * kMat * kMat;
}

Eigen::Vector3d DeviationCorrector::matrixToEulerXyz(const Eigen::Matrix3d& r, bool toDegree)
{
    double sy = std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
    bool singular = sy < 1e-6;

    Eigen::Vector3d euler;

    if (!singular)
    {
        euler(0) = std::atan2( r(2, 1), r(2, 2));
        euler(1) = std::atan2(-r(2, 0), sy);
        euler(2) = std::atan2( r(1, 0), r(0, 0));
    }
    else
    {
        euler(0) = std::atan2(-r(1, 2), r(1, 1));
        euler(1) = std::atan2(-r(2, 0), sy);
        euler(2) = 0.0;
    }

    if (toDegree)
    {
        euler *= kRadToDeg;
    }

    return euler;
}

Eigen::Matrix3d DeviationCorrector::eulerXyzToMatrix(const Eigen::Vector3d& euler, bool isDegree)
{
    double rx = isDegree ? euler(0) * kDegToRad : euler(0);
    double ry = isDegree ? euler(1) * kDegToRad : euler(1);
    double rz = isDegree ? euler(2) * kDegToRad : euler(2);

    Eigen::Matrix3d rxMat;
    rxMat << 1.0, 0.0,            0.0,
                0.0, std::cos(rx),  -std::sin(rx),
                0.0, std::sin(rx),   std::cos(rx);

    Eigen::Matrix3d ryMat;
    ryMat <<  std::cos(ry), 0.0, std::sin(ry),
                0.0,          1.0, 0.0,
                -std::sin(ry), 0.0, std::cos(ry);

    Eigen::Matrix3d rzMat;
    rzMat << std::cos(rz), -std::sin(rz), 0.0,
                std::sin(rz),  std::cos(rz), 0.0,
                0.0,           0.0,          1.0;

    // R = Rz * Ry * Rx (外旋 ZYX)
    return rzMat * ryMat * rxMat;
}

Pose6D DeviationCorrector::calculateCorrection(
    const Pose6D&        currentPose,
    const DeviationResult& deviation)
{
    if (!pImpl_->calibrated)
    {
        throw std::runtime_error(
            "Hand-eye calibration not set. Call setHandEyeCalibration() first.");
    }

    // 构造偏差矩阵 T_dev (相机坐标系)
    double drxRad = deviation.drx * kDegToRad;
    double dryRad = deviation.dry * kDegToRad;
    double drzRad = deviation.drz * kDegToRad;

    Eigen::Matrix4d tDev = Eigen::Matrix4d::Identity();
    tDev.block<3, 3>(0, 0) = eulerXyzToMatrix(
        Eigen::Vector3d(drxRad, dryRad, drzRad), false);
    tDev(0, 3) = deviation.dx;
    tDev(1, 3) = deviation.dy;
    tDev(2, 3) = deviation.dz;

    // T_B_F_new = T_B_F_cur @ T_F_C @ T_dev @ inv(T_F_C)
    Eigen::Matrix4d tBfCur  = poseToMatrix(currentPose, true);
    Eigen::Matrix4d tFcInv  = pImpl_->tFlangeCam.inverse();

    Eigen::Matrix4d tBfNew = tBfCur * pImpl_->tFlangeCam * tDev * tFcInv;

    Pose6D newPose = matrixToPose(tBfNew, true);

    // 锁定高度轴 (可配置: X=竖直方向)
    newPose.x = currentPose.x;

    return newPose;
}

Eigen::Matrix4d DeviationCorrector::computeTagInBase(
    const Pose6D&        robotPose,
    const Eigen::Vector3d& tagTvec,
    const Eigen::Vector3d& tagRvec)
{
    if (!pImpl_->calibrated)
    {
        throw std::runtime_error(
            "Hand-eye calibration not set. Call setHandEyeCalibration() first.");
    }

    // T_base_flange
    Eigen::Matrix4d tBaseFlange = poseToMatrix(robotPose, true);

    // T_cam_tag
    Eigen::Matrix3d rCamTag = rodriguesToMatrix(tagRvec);
    Eigen::Matrix4d tCamTag = Eigen::Matrix4d::Identity();
    tCamTag.block<3, 3>(0, 0) = rCamTag;
    tCamTag(0, 3) = tagTvec(0) * 1000.0; // m → mm
    tCamTag(1, 3) = tagTvec(1) * 1000.0;
    tCamTag(2, 3) = tagTvec(2) * 1000.0;

    // T_base_tag = T_base_flange @ T_flange_cam @ T_cam_tag
    return tBaseFlange * pImpl_->tFlangeCam * tCamTag;
}

std::vector<Pose6D> DeviationCorrector::propagateDeviation(
    const Eigen::Matrix4d&            tBaseTagNew,
    const std::vector<Eigen::Matrix4d>& relTransforms)
{
    std::vector<Pose6D> newPoses;
    newPoses.reserve(relTransforms.size());

    for (const auto& tTagFlange : relTransforms)
    {
        Eigen::Matrix4d tBaseFlangeNew = tBaseTagNew * tTagFlange;
        newPoses.push_back(matrixToPose(tBaseFlangeNew, true));
    }

    return newPoses;
}

bool DeviationCorrector::loadHandEyeFromFile(const std::string& filepath)
{
    try
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }

        nlohmann::json j = nlohmann::json::parse(file);
        file.close();

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();

        if (j.contains("T") && j["T"].is_array())
        {
            // 格式: {"T": [[row0], [row1], [row2], [row3]]} (2D嵌套数组)
            auto rows = j["T"];
            for (int i = 0; i < 4 && i < static_cast<int>(rows.size()); ++i)
            {
                for (int j = 0; j < 4 && j < static_cast<int>(rows[i].size()); ++j)
                {
                    matrix(i, j) = rows[i][j].get<double>();
                }
            }
        }
        else if (j.contains("result") && j["result"].is_array())
        {
            // 格式: {"result": [r00, r01, ..., r33]} (扁平数组, 行优先)
            auto arr = j["result"];
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    int idx = i * 4 + j;
                    if (idx < static_cast<int>(arr.size()))
                    {
                        matrix(i, j) = arr[idx].get<double>();
                    }
                }
            }
        }
        else
        {
            return false;
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
    ServoRecipe         currentRecipe;

    /**
     * @brief 计算各拍照点相对于标准Tag的变换矩阵
     *
     * 补偿龙门架移动引起的基座坐标系偏移:
     *   T_std_flange_i = [I | G_i - G_std] @ T_base_i_flange_i
     *   rel_transform_i = T_base_tag_std⁻¹ @ T_std_flange_i
     */
    void computeRelativeTransforms()
    {
        if (currentRecipe.tBaseTagStd.isApprox(Eigen::Matrix4d::Identity()))
        {
            return;
        }

        Eigen::Matrix4d tBaseTagInv = currentRecipe.tBaseTagStd.inverse();

        for (auto& pp : currentRecipe.photoPoints)
        {
            // 龙门架偏移量 (mm)
            double dx = static_cast<double>(pp.gantryX - currentRecipe.stdGantryX);
            double dy = static_cast<double>(pp.gantryY - currentRecipe.stdGantryY);
            double dz = static_cast<double>(pp.gantryZ - currentRecipe.stdGantryZ);

            // 基座偏移变换: 拍照点基座 → 标准基座 (仅平移)
            Eigen::Matrix4d tBaseStdBaseI = Eigen::Matrix4d::Identity();
            tBaseStdBaseI(0, 3) = dx;
            tBaseStdBaseI(1, 3) = dy;
            tBaseStdBaseI(2, 3) = dz;

            // 将拍照点机械臂位姿转换到标准基座系
            Eigen::Matrix4d tBaseFlange = DeviationCorrector::poseToMatrix(pp.pose, true);
            Eigen::Matrix4d tStdFlange  = tBaseStdBaseI * tBaseFlange;

            // 相对于标准Tag的变换
            pp.rel_transform = tBaseTagInv * tStdFlange;
        }
    }
};

MultiPointServo::MultiPointServo()
    : pImpl_(new Impl())
{
}

MultiPointServo::MultiPointServo(const std::string& handEyeFile)
    : pImpl_(new Impl())
{
    pImpl_->corrector.loadHandEyeFromFile(handEyeFile);
}

MultiPointServo::~MultiPointServo() = default;

ServoRecipe MultiPointServo::startTeaching(const std::string& name)
{
    pImpl_->currentRecipe = ServoRecipe();

    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    pImpl_->currentRecipe.createdTime =
        std::chrono::duration<double>(duration).count();

    if (name.empty())
    {
        std::ostringstream oss;
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTm;
        localtime_s(&localTm, &t);
        oss << "Recipe_" << std::put_time(&localTm, "%Y%m%d_%H%M%S");
        pImpl_->currentRecipe.name = oss.str();
    }
    else
    {
        pImpl_->currentRecipe.name = name;
    }

    pImpl_->currentRecipe.id = "recipe_"
        + std::to_string(static_cast<long long>(pImpl_->currentRecipe.createdTime));

    return pImpl_->currentRecipe;
}

bool MultiPointServo::recordStandardPoint(
    const Pose6D&       robotPose,
    const TagDetection& tagResult,
    float               gantryX,
    float               gantryY,
    float               gantryZ)
{
    pImpl_->currentRecipe.stdRobotPose  = robotPose;
    pImpl_->currentRecipe.stdTagData    = tagResult;
    pImpl_->currentRecipe.stdGantryX    = gantryX;
    pImpl_->currentRecipe.stdGantryY    = gantryY;
    pImpl_->currentRecipe.stdGantryZ    = gantryZ;

    try
    {
        pImpl_->currentRecipe.tBaseTagStd = pImpl_->corrector.computeTagInBase(
            robotPose, tagResult.tvec, tagResult.rvec);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

int MultiPointServo::addPhotoPoint(
    const std::string&  name,
    const Pose6D&       robotPose,
    float               gantryX,
    float               gantryY,
    float               gantryZ)
{
    PhotoPoint pp;
    pp.name    = name;
    pp.pose    = robotPose;
    pp.gantryX = gantryX;
    pp.gantryY = gantryY;
    pp.gantryZ = gantryZ;
    pp.rel_transform = Eigen::Matrix4d::Identity();

    pImpl_->currentRecipe.photoPoints.push_back(pp);

    return static_cast<int>(pImpl_->currentRecipe.photoPoints.size());
}

ServoRecipe MultiPointServo::finishTeaching()
{
    if (pImpl_->currentRecipe.tBaseTagStd.isApprox(Eigen::Matrix4d::Identity()))
    {
        throw std::runtime_error(
            "Standard point not recorded. Call recordStandardPoint() first.");
    }

    pImpl_->computeRelativeTransforms();

    return pImpl_->currentRecipe;
}

std::vector<std::pair<std::string, Pose6D>> MultiPointServo::computeNewPoses(
    const Pose6D&       robotPose,
    const TagDetection& tagResult,
    float               currentGantryX,
    float               currentGantryY,
    float               currentGantryZ)
{
    std::vector<std::pair<std::string, Pose6D>> results;

    // Tag 在当前基座系下的位姿
    Eigen::Matrix4d tBaseTagNew = pImpl_->corrector.computeTagInBase(
        robotPose, tagResult.tvec, tagResult.rvec);

    // 将 Tag 位姿从当前基座系转换到标准基座系
    double dx = static_cast<double>(currentGantryX - pImpl_->currentRecipe.stdGantryX);
    double dy = static_cast<double>(currentGantryY - pImpl_->currentRecipe.stdGantryY);
    double dz = static_cast<double>(currentGantryZ - pImpl_->currentRecipe.stdGantryZ);

    Eigen::Matrix4d tBaseOffset = Eigen::Matrix4d::Identity();
    tBaseOffset(0, 3) = dx;
    tBaseOffset(1, 3) = dy;
    tBaseOffset(2, 3) = dz;
    tBaseTagNew = tBaseOffset * tBaseTagNew;

    std::vector<Eigen::Matrix4d> relTransforms;
    relTransforms.reserve(pImpl_->currentRecipe.photoPoints.size());
    for (const auto& pp : pImpl_->currentRecipe.photoPoints)
    {
        relTransforms.push_back(pp.rel_transform);
    }

    std::vector<Pose6D> newPoses = pImpl_->corrector.propagateDeviation(
        tBaseTagNew, relTransforms);

    for (size_t i = 0; i < newPoses.size(); ++i)
    {
        results.emplace_back(
            pImpl_->currentRecipe.photoPoints[i].name,
            newPoses[i]);
    }

    return results;
}

Pose6D MultiPointServo::adjustForGantry(
    const Pose6D& poseStdFrame,
    float         stdGantryX,
    float         stdGantryY,
    float         stdGantryZ,
    float         targetGantryX,
    float         targetGantryY,
    float         targetGantryZ)
{
    Pose6D result = poseStdFrame;
    result.x -= static_cast<double>(targetGantryX - stdGantryX);
    result.y -= static_cast<double>(targetGantryY - stdGantryY);
    result.z -= static_cast<double>(targetGantryZ - stdGantryZ);
    return result;
}

// ==================== 配方 JSON 持久化 (nlohmann/json) ====================

bool MultiPointServo::saveRecipe(const std::string& filepath) const
{
    try
    {
        const auto& r = pImpl_->currentRecipe;

        nlohmann::json j;
        j["id"]          = r.id;
        j["name"]        = r.name;
        j["createdTime"] = r.createdTime;
        j["description"] = r.description;

        // 标准点机械臂位姿
        j["stdRobotPose"] = {r.stdRobotPose.x, r.stdRobotPose.y, r.stdRobotPose.z,
                                r.stdRobotPose.rx, r.stdRobotPose.ry, r.stdRobotPose.rz};

        // 标准点龙门架位置
        j["stdGantryX"] = r.stdGantryX;
        j["stdGantryY"] = r.stdGantryY;
        j["stdGantryZ"] = r.stdGantryZ;

        // T_base_tag_std (4x4矩阵 → 2D嵌套数组)
        nlohmann::json tRows = nlohmann::json::array();
        for (int i = 0; i < 4; ++i)
        {
            nlohmann::json row = nlohmann::json::array();
            for (int j = 0; j < 4; ++j)
            {
                row.push_back(r.tBaseTagStd(i, j));
            }
            tRows.push_back(row);
        }
        j["tBaseTagStd"] = tRows;

        // 拍照点列表
        nlohmann::json ptsArray = nlohmann::json::array();
        for (const auto& pp : r.photoPoints)
        {
            nlohmann::json pt;
            pt["name"]  = pp.name;
            pt["pose"]  = {pp.pose.x, pp.pose.y, pp.pose.z,
                            pp.pose.rx, pp.pose.ry, pp.pose.rz};
            pt["gantryX"] = pp.gantryX;
            pt["gantryY"] = pp.gantryY;
            pt["gantryZ"] = pp.gantryZ;

            // rel_transform
            nlohmann::json relRows = nlohmann::json::array();
            for (int i = 0; i < 4; ++i)
            {
                nlohmann::json row = nlohmann::json::array();
                for (int j = 0; j < 4; ++j)
                {
                    row.push_back(pp.rel_transform(i, j));
                }
                relRows.push_back(row);
            }
            pt["relTransform"] = relRows;

            if (!pp.snapshotPath.empty())
            {
                pt["snapshotPath"] = pp.snapshotPath;
            }

            ptsArray.push_back(pt);
        }
        j["photoPoints"] = ptsArray;

        if (!r.handEyeFile.empty())
        {
            j["handEyeFile"] = r.handEyeFile;
        }

        std::ofstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }
        file << std::setw(2) << j << std::endl;
        file.close();

        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool MultiPointServo::loadRecipe(const std::string& filepath)
{
    try
    {
        std::ifstream file(filepath);
        if (!file.is_open())
        {
            return false;
        }

        nlohmann::json j = nlohmann::json::parse(file);
        file.close();

        ServoRecipe recipe;

        // 基本字段
        recipe.id          = j.value("id", "");
        recipe.name        = j.value("name", "");
        recipe.createdTime = j.value("createdTime", 0.0);
        recipe.description = j.value("description", "");
        recipe.handEyeFile = j.value("handEyeFile", "");

        // 标准点
        if (j.contains("stdRobotPose") && j["stdRobotPose"].size() >= 6)
        {
            auto& arr = j["stdRobotPose"];
            recipe.stdRobotPose = Pose6D(
                arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>(),
                arr[3].get<double>(), arr[4].get<double>(), arr[5].get<double>());
        }
        recipe.stdGantryX = j.value("stdGantryX", 0.0f);
        recipe.stdGantryY = j.value("stdGantryY", 0.0f);
        recipe.stdGantryZ = j.value("stdGantryZ", 0.0f);

        // T_base_tag_std
        if (j.contains("tBaseTagStd") && j["tBaseTagStd"].is_array())
        {
            auto& rows = j["tBaseTagStd"];
            for (int i = 0; i < 4 && i < static_cast<int>(rows.size()); ++i)
            {
                for (int j = 0; j < 4 && j < static_cast<int>(rows[i].size()); ++j)
                {
                    recipe.tBaseTagStd(i, j) = rows[i][j].get<double>();
                }
            }
        }

        // 拍照点列表
        if (j.contains("photoPoints") && j["photoPoints"].is_array())
        {
            for (const auto& ptJson : j["photoPoints"])
            {
                PhotoPoint pp;
                pp.name = ptJson.value("name", "");

                if (ptJson.contains("pose") && ptJson["pose"].size() >= 6)
                {
                    auto& arr = ptJson["pose"];
                    pp.pose = Pose6D(
                        arr[0].get<double>(), arr[1].get<double>(),
                        arr[2].get<double>(),
                        arr[3].get<double>(), arr[4].get<double>(),
                        arr[5].get<double>());
                }

                pp.gantryX = ptJson.value("gantryX", 0.0f);
                pp.gantryY = ptJson.value("gantryY", 0.0f);
                pp.gantryZ = ptJson.value("gantryZ", 0.0f);

                pp.snapshotPath = ptJson.value("snapshotPath", "");

                // rel_transform
                if (ptJson.contains("relTransform")
                    && ptJson["relTransform"].is_array())
                {
                    auto& rows = ptJson["relTransform"];
                    for (int i = 0; i < 4 && i < static_cast<int>(rows.size()); ++i)
                    {
                        for (int j = 0;
                                j < 4 && j < static_cast<int>(rows[i].size());
                                ++j)
                        {
                            pp.rel_transform(i, j) = rows[i][j].get<double>();
                        }
                    }
                }

                recipe.photoPoints.push_back(pp);
            }
        }

        pImpl_->currentRecipe = std::move(recipe);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

const ServoRecipe& MultiPointServo::getCurrentRecipe() const
{
    return pImpl_->currentRecipe;
}

void MultiPointServo::setHandEyeCalibration(const Eigen::Matrix4d& tFlangeCam)
{
    pImpl_->corrector.setHandEyeCalibration(tFlangeCam);
}

bool MultiPointServo::loadHandEyeFromFile(const std::string& filepath)
{
    return pImpl_->corrector.loadHandEyeFromFile(filepath);
}

} // namespace vision_servo
