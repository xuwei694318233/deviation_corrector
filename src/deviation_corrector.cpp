/**
 * @file deviation_corrector.cpp
 * @brief AGV 统一偏移模型 — 视觉伺服纠偏实现
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

constexpr double kPi       = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kEpsilon  = 1e-10;
constexpr double kMToMm    = 1000.0;   ///< 米 → 毫米

// ==================== DeviationCorrector ====================

struct DeviationCorrector::Impl
{
    Eigen::Matrix4d tFlangeCam{Eigen::Matrix4d::Identity()};
    bool calibrated = false;
};

DeviationCorrector::DeviationCorrector() : pImpl_(std::make_unique<Impl>()) {}
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

    const double rx = isDegree ? pose.rx * kDegToRad : pose.rx;
    const double ry = isDegree ? pose.ry * kDegToRad : pose.ry;
    const double rz = isDegree ? pose.rz * kDegToRad : pose.rz;

    t.block<3, 3>(0, 0) = eulerXyzToMatrix(Eigen::Vector3d(rx, ry, rz), false);
    return t;
}

Pose6D DeviationCorrector::matrixToPose(const Eigen::Matrix4d& matrix, bool toDegree)
{
    Pose6D pose;
    pose.x = matrix(0, 3);
    pose.y = matrix(1, 3);
    pose.z = matrix(2, 3);

    const Eigen::Matrix3d r = matrix.block<3, 3>(0, 0);
    const Eigen::Vector3d euler = matrixToEulerXyz(r, toDegree);
    pose.rx = euler(0);
    pose.ry = euler(1);
    pose.rz = euler(2);

    return pose;
}

Eigen::Matrix3d DeviationCorrector::rodriguesToMatrix(const Eigen::Vector3d& rvec)
{
    const double theta = rvec.norm();
    if (theta < kEpsilon)
    {
        return Eigen::Matrix3d::Identity();
    }

    const Eigen::Vector3d k = rvec / theta;
    Eigen::Matrix3d kMat;
    kMat << 0, -k(2),  k(1),
            k(2), 0, -k(0),
           -k(1), k(0), 0;

    return Eigen::Matrix3d::Identity()
        + std::sin(theta) * kMat
        + (1.0 - std::cos(theta)) * kMat * kMat;
}

Eigen::Vector3d DeviationCorrector::matrixToEulerXyz(const Eigen::Matrix3d& r, bool toDegree)
{
    const double sy = std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
    const bool singular = sy < kEpsilon;

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
    const double rx = isDegree ? euler(0) * kDegToRad : euler(0);
    const double ry = isDegree ? euler(1) * kDegToRad : euler(1);
    const double rz = isDegree ? euler(2) * kDegToRad : euler(2);

    Eigen::Matrix3d rxMat;
    rxMat << 1.0, 0.0, 0.0,
             0.0, std::cos(rx), -std::sin(rx),
             0.0, std::sin(rx),  std::cos(rx);

    Eigen::Matrix3d ryMat;
    ryMat <<  std::cos(ry), 0.0, std::sin(ry),
              0.0,          1.0, 0.0,
             -std::sin(ry), 0.0, std::cos(ry);

    Eigen::Matrix3d rzMat;
    rzMat << std::cos(rz), -std::sin(rz), 0.0,
             std::sin(rz),  std::cos(rz), 0.0,
             0.0,           0.0,          1.0;

    return rzMat * ryMat * rxMat;
}

Pose6D DeviationCorrector::calculateCorrection(
    const Pose6D& currentPose, const DeviationResult& deviation)
{
    if (!pImpl_->calibrated)
    {
        throw std::runtime_error("Hand-eye calibration not set.");
    }

    const double drxRad = deviation.drx * kDegToRad;
    const double dryRad = deviation.dry * kDegToRad;
    const double drzRad = deviation.drz * kDegToRad;

    Eigen::Matrix4d tDev = Eigen::Matrix4d::Identity();
    tDev.block<3, 3>(0, 0) = eulerXyzToMatrix(Eigen::Vector3d(drxRad, dryRad, drzRad), false);
    tDev(0, 3) = deviation.dx;
    tDev(1, 3) = deviation.dy;
    tDev(2, 3) = deviation.dz;

    const Eigen::Matrix4d tBfCur  = poseToMatrix(currentPose, true);
    const Eigen::Matrix4d tFcInv  = pImpl_->tFlangeCam.inverse();
    const Eigen::Matrix4d tBfNew  = tBfCur * pImpl_->tFlangeCam * tDev * tFcInv;

    Pose6D newPose = matrixToPose(tBfNew, true);
    newPose.x = currentPose.x;  // 锁定高度轴
    return newPose;
}

Eigen::Matrix4d DeviationCorrector::computeTagInBase(
    const Pose6D& robotPose,
    const Eigen::Vector3d& tagTvec,
    const Eigen::Vector3d& tagRvec)
{
    if (!pImpl_->calibrated)
    {
        throw std::runtime_error("Hand-eye calibration not set.");
    }

    const Eigen::Matrix4d tBaseFlange = poseToMatrix(robotPose, true);
    const Eigen::Matrix3d rCamTag = rodriguesToMatrix(tagRvec);

    Eigen::Matrix4d tCamTag = Eigen::Matrix4d::Identity();
    tCamTag.block<3, 3>(0, 0) = rCamTag;
    tCamTag(0, 3) = tagTvec(0) * kMToMm;
    tCamTag(1, 3) = tagTvec(1) * kMToMm;
    tCamTag(2, 3) = tagTvec(2) * kMToMm;

    return tBaseFlange * pImpl_->tFlangeCam * tCamTag;
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
            const auto& rows = j["T"];
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
            const auto& arr = j["result"];
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    const int idx = i * 4 + j;
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

// ==================== MultiPointServo ====================

struct MultiPointServo::Impl
{
    DeviationCorrector corrector;
    ServoRecipe         currentRecipe;
};

MultiPointServo::MultiPointServo() : pImpl_(std::make_unique<Impl>()) {}

MultiPointServo::MultiPointServo(const std::string& handEyeFile)
    : pImpl_(std::make_unique<Impl>())
{
    pImpl_->corrector.loadHandEyeFromFile(handEyeFile);
}

MultiPointServo::~MultiPointServo() = default;

ServoRecipe MultiPointServo::startTeaching(const std::string& name)
{
    pImpl_->currentRecipe = ServoRecipe();

    const auto now = std::chrono::system_clock::now();
    const auto duration = now.time_since_epoch();
    pImpl_->currentRecipe.createdTime = std::chrono::duration<double>(duration).count();

    if (name.empty())
    {
        const std::time_t t = std::chrono::system_clock::to_time_t(now);
        struct tm localTm;
        localtime_s(&localTm, &t);
        std::ostringstream oss;
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
    const Pose6D& robotPose, const TagDetection& tagResult,
    float gantryX, float gantryY, float gantryZ)
{
    pImpl_->currentRecipe.stdRobotPose = robotPose;
    pImpl_->currentRecipe.stdGantryX   = gantryX;
    pImpl_->currentRecipe.stdGantryY   = gantryY;
    pImpl_->currentRecipe.stdGantryZ   = gantryZ;
    pImpl_->currentRecipe.stdTagData   = tagResult;

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

int MultiPointServo::addPhotoPoint(const std::string& name, const Pose6D& robotPose,
    float gantryX, float gantryY, float gantryZ)
{
    PhotoPoint pp;
    pp.name    = name;
    pp.pose    = robotPose;
    pp.gantryX = gantryX;
    pp.gantryY = gantryY;
    pp.gantryZ = gantryZ;
    pImpl_->currentRecipe.photoPoints.emplace_back(std::move(pp));
    return static_cast<int>(pImpl_->currentRecipe.photoPoints.size());
}

ServoRecipe MultiPointServo::finishTeaching()
{
    if (pImpl_->currentRecipe.tBaseTagStd.isApprox(Eigen::Matrix4d::Identity()))
    {
        throw std::runtime_error("Standard point not recorded.");
    }
    return pImpl_->currentRecipe;
}

std::vector<std::pair<std::string, Pose6D>> MultiPointServo::computeNewPoses(
    const Pose6D& robotPose, const TagDetection& tagResult,
    float curGantryX, float curGantryY, float curGantryZ)
{
    std::vector<std::pair<std::string, Pose6D>> results;

    const Eigen::Matrix4d tBaseTagCur = pImpl_->corrector.computeTagInBase(
        robotPose, tagResult.tvec, tagResult.rvec);

    // T_cur 在 G_cur 系，转到 G_std 系再算 ΔT
    Eigen::Matrix4d curToStd = Eigen::Matrix4d::Identity();
    curToStd(0, 3) = static_cast<double>(curGantryX - pImpl_->currentRecipe.stdGantryX);
    curToStd(1, 3) = static_cast<double>(curGantryY - pImpl_->currentRecipe.stdGantryY);
    curToStd(2, 3) = static_cast<double>(curGantryZ - pImpl_->currentRecipe.stdGantryZ);
    const Eigen::Matrix4d tBaseTagCurStd = curToStd * tBaseTagCur;

    const Eigen::Matrix4d deltaT = tBaseTagCurStd * pImpl_->currentRecipe.tBaseTagStd.inverse();

    for (const auto& pp : pImpl_->currentRecipe.photoPoints)
    {
        // P_i(G_i系) → G_std系 → ΔT → 转回G_i系
        const double dx = static_cast<double>(pp.gantryX - pImpl_->currentRecipe.stdGantryX);
        const double dy = static_cast<double>(pp.gantryY - pImpl_->currentRecipe.stdGantryY);
        const double dz = static_cast<double>(pp.gantryZ - pImpl_->currentRecipe.stdGantryZ);
        Eigen::Matrix4d toStd = Eigen::Matrix4d::Identity();
        toStd(0, 3) = dx;
        toStd(1, 3) = dy;
        toStd(2, 3) = dz;

        const Eigen::Matrix4d tOld = DeviationCorrector::poseToMatrix(pp.pose, true);
        const Eigen::Matrix4d tNew = toStd.inverse() * deltaT * toStd * tOld;

        results.emplace_back(pp.name, DeviationCorrector::matrixToPose(tNew, true));
    }

    return results;
}

// ==================== 配方 JSON 持久化 ====================

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

        j["stdRobotPose"] = {
            r.stdRobotPose.x,  r.stdRobotPose.y,  r.stdRobotPose.z,
            r.stdRobotPose.rx, r.stdRobotPose.ry, r.stdRobotPose.rz
        };
        j["stdGantryX"] = r.stdGantryX;
        j["stdGantryY"] = r.stdGantryY;
        j["stdGantryZ"] = r.stdGantryZ;

        // tBaseTagStd 4x4 → 2D 数组
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
            pt["name"] = pp.name;
            pt["pose"] = {pp.pose.x,  pp.pose.y,  pp.pose.z,
                          pp.pose.rx, pp.pose.ry, pp.pose.rz};
            pt["gantryX"] = pp.gantryX;
            pt["gantryY"] = pp.gantryY;
            pt["gantryZ"] = pp.gantryZ;
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

        ServoRecipe r;
        r.id           = j.value("id", "");
        r.name         = j.value("name", "");
        r.createdTime  = j.value("createdTime", 0.0);
        r.description  = j.value("description", "");
        r.handEyeFile  = j.value("handEyeFile", "");
        r.stdGantryX   = j.value("stdGantryX", 0.0f);
        r.stdGantryY   = j.value("stdGantryY", 0.0f);
        r.stdGantryZ   = j.value("stdGantryZ", 0.0f);

        if (j.contains("stdRobotPose") && j["stdRobotPose"].size() >= 6)
        {
            const auto& arr = j["stdRobotPose"];
            r.stdRobotPose = Pose6D(
                arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>(),
                arr[3].get<double>(), arr[4].get<double>(), arr[5].get<double>());
        }

        if (j.contains("tBaseTagStd") && j["tBaseTagStd"].is_array())
        {
            const auto& rows = j["tBaseTagStd"];
            for (int i = 0; i < 4 && i < static_cast<int>(rows.size()); ++i)
            {
                for (int j = 0; j < 4 && j < static_cast<int>(rows[i].size()); ++j)
                {
                    r.tBaseTagStd(i, j) = rows[i][j].get<double>();
                }
            }
        }

        if (j.contains("photoPoints") && j["photoPoints"].is_array())
        {
            for (const auto& ptJson : j["photoPoints"])
            {
                PhotoPoint pp;
                pp.name = ptJson.value("name", "");

                if (ptJson.contains("pose") && ptJson["pose"].size() >= 6)
                {
                    const auto& arr = ptJson["pose"];
                    pp.pose = Pose6D(
                        arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>(),
                        arr[3].get<double>(), arr[4].get<double>(), arr[5].get<double>());
                }

                pp.gantryX     = ptJson.value("gantryX", 0.0f);
                pp.gantryY     = ptJson.value("gantryY", 0.0f);
                pp.gantryZ     = ptJson.value("gantryZ", 0.0f);
                pp.snapshotPath = ptJson.value("snapshotPath", "");
                r.photoPoints.push_back(pp);
            }
        }

        pImpl_->currentRecipe = std::move(r);
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
