/**
 * @file test_deviation_corrector.cpp
 * @brief AGV 统一偏移模型 — 单元测试 (C++17)
 */

#include "deviation_corrector.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>

using namespace vision_servo;

namespace
{

constexpr double kTol     = 0.01;     ///< 浮点比较容差
constexpr float  kGantryX = 500.0f;   ///< 测试用龙门架 X (mm)
constexpr float  kGantryY = 300.0f;   ///< 测试用龙门架 Y (mm)
constexpr float  kGantryZ = 200.0f;   ///< 测试用龙门架 Z (mm)
constexpr double kTagDist = 0.5;      ///< 测试用 Tag 距离 (m)

void TestPoseRoundtrip()
{
    std::cout << "\n== Test 1: Pose Roundtrip ==\n";

    const Pose6D pose(100.0, 200.0, 300.0, 10.0, -20.0, 45.0);
    const Eigen::Matrix4d mat = DeviationCorrector::poseToMatrix(pose, true);
    const Pose6D back = DeviationCorrector::matrixToPose(mat, true);

    const double posErr = std::abs(pose.x - back.x)
        + std::abs(pose.y - back.y) + std::abs(pose.z - back.z);
    const double rotErr = std::abs(pose.rx - back.rx)
        + std::abs(pose.ry - back.ry) + std::abs(pose.rz - back.rz);

    assert(posErr < kTol);
    assert(rotErr < kTol);
    std::cout << "PASSED\n";
}

void TestRodrigues()
{
    std::cout << "\n== Test 2: Rodrigues ==\n";

    constexpr double kHalfPi = 1.5707963267948966;
    const Eigen::Vector3d rvec(0.0, 0.0, kHalfPi);
    const Eigen::Vector3d rotated = DeviationCorrector::rodriguesToMatrix(rvec)
        * Eigen::Vector3d(1.0, 0.0, 0.0);

    assert(std::abs(rotated(0)) < kTol);
    assert(std::abs(rotated(1) - 1.0) < kTol);
    std::cout << "PASSED\n";
}

void TestSingleCorrection()
{
    std::cout << "\n== Test 3: Single Correction ==\n";

    DeviationCorrector corrector;

    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(0, 3) = 50.0;
    tFlangeCam(2, 3) = 100.0;
    corrector.setHandEyeCalibration(tFlangeCam);

    const Pose6D curPose(500.0, 300.0, 400.0, 0.0, -90.0, 0.0);
    const auto dev = DeviationResult::xyPlane(-50.0, 0.0, 0.0);

    const Pose6D target = corrector.calculateCorrection(curPose, dev);
    // XY 平面偏差 -50mm 应体现在目标位姿 X 分量上
    assert(std::abs(target.x - 500.0) < kTol);
    std::cout << "PASSED\n";
}

void TestComputeTagInBase()
{
    std::cout << "\n== Test 4: ComputeTagInBase ==\n";

    DeviationCorrector corrector;

    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(2, 3) = 150.0;
    corrector.setHandEyeCalibration(tFlangeCam);

    const Pose6D robotPose(400.0, 200.0, 500.0, 0.0, -90.0, 0.0);
    const Eigen::Vector3d tagTvec(0.0, 0.0, kTagDist);
    const Eigen::Vector3d tagRvec(0.0, 0.0, 0.0);

    const Eigen::Matrix4d tBaseTag = corrector.computeTagInBase(
        robotPose, tagTvec, tagRvec);
    const Eigen::Vector3d tagPos = tBaseTag.block<3, 1>(0, 3);

    std::cout << "  Tag in Base: [" << tagPos.transpose() << "] mm\n";
    assert(!tBaseTag.isIdentity());
    std::cout << "PASSED\n";
}

void TestDeltaTModel()
{
    std::cout << "\n== Test 5: DeltaT Model ==\n";

    DeviationCorrector corrector;

    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(2, 3) = 150.0;
    corrector.setHandEyeCalibration(tFlangeCam);

    const Eigen::Matrix4d tStd = corrector.computeTagInBase(
        Pose6D(400.0, 200.0, 500.0, 0.0, -90.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, kTagDist),
        Eigen::Vector3d(0.0, 0.0, 0.0));

    const Eigen::Matrix4d tCur = corrector.computeTagInBase(
        Pose6D(405.0, 203.0, 500.0, 0.0, -90.0, 0.0),
        Eigen::Vector3d(0.01, 0.005, 0.5),
        Eigen::Vector3d(0.0, 0.0, 0.01));

    const Eigen::Matrix4d deltaT = tCur * tStd.inverse();
    assert(!deltaT.isIdentity());
    std::cout << "PASSED\n";
}

void TestFullWorkflow()
{
    std::cout << "\n== Test 6: Full Workflow ==\n";

    MultiPointServo servo;

    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(2, 3) = 150.0;
    servo.setHandEyeCalibration(tFlangeCam);

    servo.startTeaching("T");

    TagDetection stdTag;
    stdTag.tvec = Eigen::Vector3d(0.0, 0.0, kTagDist);
    stdTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);

    assert(servo.recordStandardPoint(
        Pose6D(400.0, 200.0, 500.0, 0.0, -90.0, 0.0),
        stdTag, kGantryX, kGantryY, kGantryZ));

    servo.addPhotoPoint("Front", Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0),
        kGantryX, kGantryY, kGantryZ);
    servo.addPhotoPoint("Side",  Pose6D(350.0, 250.0, 480.0, 0.0, -90.0, 0.0),
        kGantryX, kGantryY, kGantryZ);

    servo.finishTeaching();
    servo.saveRecipe("test_recipe.json");
    std::cout << "  Teaching done\n";

    TagDetection prodTag;
    prodTag.tvec = Eigen::Vector3d(0.01, 0.005, 0.5);
    prodTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.01);

    const auto newPoses = servo.computeNewPoses(
        Pose6D(405.0, 203.0, 500.0, 0.0, -90.0, 0.0),
        prodTag, kGantryX, kGantryY, kGantryZ);

    assert(newPoses.size() == 2);
    std::cout << "PASSED\n";
}

void TestAgvNoMove()
{
    std::cout << "\n== Test 7: AGV Not Moved ==\n";

    MultiPointServo servo;

    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(2, 3) = 150.0;
    servo.setHandEyeCalibration(tFlangeCam);

    servo.startTeaching("N");

    TagDetection stdTag;
    stdTag.tvec = Eigen::Vector3d(0.0, 0.0, kTagDist);
    stdTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);

    servo.recordStandardPoint(
        Pose6D(400.0, 200.0, 500.0, 0.0, -90.0, 0.0),
        stdTag, kGantryX, kGantryY, kGantryZ);
    servo.addPhotoPoint("P1",
        Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0),
        kGantryX, kGantryY, kGantryZ);
    servo.finishTeaching();

    // AGV 未移动 → 补偿位姿应等于示教位姿
    const auto newPoses = servo.computeNewPoses(
        Pose6D(400.0, 200.0, 500.0, 0.0, -90.0, 0.0),
        stdTag, kGantryX, kGantryY, kGantryZ);

    assert(newPoses.size() == 1);
    assert(std::abs(newPoses[0].second.x - 450.0) < kTol);
    assert(std::abs(newPoses[0].second.y - 200.0) < kTol);
    assert(std::abs(newPoses[0].second.z - 480.0) < kTol);
    std::cout << "PASSED\n";
}

void TestRecipeSaveLoad()
{
    std::cout << "\n== Test 8: Recipe Save/Load ==\n";

    MultiPointServo servo1;

    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(2, 3) = 150.0;
    servo1.setHandEyeCalibration(tFlangeCam);

    servo1.startTeaching("L");

    TagDetection stdTag;
    stdTag.tvec = Eigen::Vector3d(0.0, 0.0, kTagDist);
    stdTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);

    servo1.recordStandardPoint(
        Pose6D(400.0, 200.0, 500.0, 0.0, -90.0, 0.0),
        stdTag, kGantryX, kGantryY, kGantryZ);
    servo1.addPhotoPoint("P1",
        Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0),
        kGantryX, kGantryY, kGantryZ);
    servo1.addPhotoPoint("P2",
        Pose6D(350.0, 250.0, 480.0, 0.0, -90.0, 0.0),
        kGantryX, kGantryY, kGantryZ);
    servo1.finishTeaching();
    servo1.saveRecipe("test_loadsave.json");

    MultiPointServo servo2;
    servo2.setHandEyeCalibration(tFlangeCam);
    assert(servo2.loadRecipe("test_loadsave.json"));

    const auto& recipe = servo2.getCurrentRecipe();
    assert(recipe.name == "L");
    assert(recipe.photoPoints.size() == 2);
    assert(recipe.photoPoints[0].name == "P1");
    std::cout << "PASSED\n";
}

void TestHandEyeFormats()
{
    std::cout << "\n== Test 9: HandEye JSON Formats ==\n";

    // 格式 1: {"T": [[row],...]}
    {
        std::ofstream file("test_he_f1.json");
        file << "{\"T\":[[1,0,0,50],[0,1,0,0],[0,0,1,100],[0,0,0,1]]}\n";
        file.close();

        DeviationCorrector corrector;
        assert(corrector.loadHandEyeFromFile("test_he_f1.json"));
        assert(std::abs(corrector.getHandEyeCalibration()(0, 3) - 50.0) < kTol);
        std::cout << "  Format 1 (T key) OK\n";
    }

    // 格式 2: {"result": [16 floats]}
    {
        std::ofstream file("test_he_f2.json");
        file << "{\"result\":[1,0,0,50,0,1,0,0,0,0,1,100,0,0,0,1]}\n";
        file.close();

        DeviationCorrector corrector;
        assert(corrector.loadHandEyeFromFile("test_he_f2.json"));
        assert(std::abs(corrector.getHandEyeCalibration()(0, 3) - 50.0) < kTol);
        std::cout << "  Format 2 (result key) OK\n";
    }

    std::cout << "PASSED\n";
}

} // anonymous namespace

int main()
{
    std::cout << "========================================\n";
    std::cout << "  Deviation Corrector Tests (AGV ΔT)\n";
    std::cout << "========================================\n";

    try
    {
        TestPoseRoundtrip();
        TestRodrigues();
        TestSingleCorrection();
        TestComputeTagInBase();
        TestDeltaTModel();
        TestFullWorkflow();
        TestAgvNoMove();
        TestRecipeSaveLoad();
        TestHandEyeFormats();

        std::cout << "\n========================================\n";
        std::cout << "  All 9 Tests PASSED!\n";
        std::cout << "========================================\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAILED: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
