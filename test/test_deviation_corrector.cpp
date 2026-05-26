/**
 * @file test_deviation_corrector.cpp
 * @brief 纠偏算法动态库单元测试
 */

#include "deviation_corrector.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cassert>

using namespace vision_servo;

// 辅助: 打印位姿
void printPose(const std::string& label, const Pose6D& pose)
{
    std::cout << label << ":\n";
    std::cout << "  Position: X=" << std::fixed << std::setprecision(3)
              << pose.x << " Y=" << pose.y << " Z=" << pose.z << " mm\n";
    std::cout << "  Rotation: RX=" << pose.rx << " RY=" << pose.ry
              << " RZ=" << pose.rz << " deg\n";
}

// 辅助: 打印矩阵
void printMatrix(const std::string& label, const Eigen::Matrix4d& matrix)
{
    std::cout << label << ":\n";
    for (int i = 0; i < 4; ++i)
    {
        std::cout << "  [";
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::fixed << std::setprecision(6) << std::setw(12)
                      << matrix(i, j);
        }
        std::cout << " ]\n";
    }
}

void testPoseMatrixRoundtrip()
{
    std::cout << "\n========== Test 1: Pose-Matrix Roundtrip ==========\n";

    Pose6D pose(100.0, 200.0, 300.0, 10.0, -20.0, 45.0);
    printPose("Original", pose);

    Eigen::Matrix4d m = DeviationCorrector::poseToMatrix(pose, true);
    Pose6D back = DeviationCorrector::matrixToPose(m, true);
    printPose("Recovered", back);

    double posErr = std::abs(pose.x - back.x) + std::abs(pose.y - back.y)
                  + std::abs(pose.z - back.z);
    double rotErr = std::abs(pose.rx - back.rx) + std::abs(pose.ry - back.ry)
                  + std::abs(pose.rz - back.rz);

    assert(posErr < 0.001);
    assert(rotErr < 0.001);
    std::cout << "PASSED\n";
}

void testRodrigues()
{
    std::cout << "\n========== Test 2: Rodrigues ==========\n";

    constexpr double kHalfPi = 1.5707963267948966;
    Eigen::Vector3d rvec(0.0, 0.0, kHalfPi);
    Eigen::Matrix3d r = DeviationCorrector::rodriguesToMatrix(rvec);
    Eigen::Vector3d rotated = r * Eigen::Vector3d(1.0, 0.0, 0.0);

    std::cout << "Rotated X-axis: [" << rotated.transpose() << "]\n";
    assert(std::abs(rotated(0)) < 0.001);
    assert(std::abs(rotated(1) - 1.0) < 0.001);
    std::cout << "PASSED\n";
}

void testSingleCorrection()
{
    std::cout << "\n========== Test 3: Single Point Correction ==========\n";

    DeviationCorrector corrector;

    Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
    tFc(0, 3) =  50.0;
    tFc(2, 3) = 100.0;
    corrector.setHandEyeCalibration(tFc);

    Pose6D cur(500.0, 300.0, 400.0, 0.0, -90.0, 0.0);
    auto dev = DeviationResult::xyPlane(-50.0, 0.0, 0.0);

    Pose6D target = corrector.calculateCorrection(cur, dev);
    printPose("Target", target);
    std::cout << "PASSED\n";
}

void testComputeTagInBase()
{
    std::cout << "\n========== Test 4: Compute Tag In Base ==========\n";

    DeviationCorrector corrector;

    Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
    tFc(2, 3) = 150.0;
    corrector.setHandEyeCalibration(tFc);

    Pose6D robotPose(400.0, 200.0, 500.0, 0.0, -90.0, 0.0);
    Eigen::Vector3d tagTvec(0.0, 0.0, 0.5);
    Eigen::Vector3d tagRvec(0.0, 0.0, 0.0);

    Eigen::Matrix4d tBaseTag = corrector.computeTagInBase(
        robotPose, tagTvec, tagRvec);
    printMatrix("Tag in Base", tBaseTag);

    Eigen::Vector3d tagPos = tBaseTag.block<3, 1>(0, 3);
    std::cout << "Tag Position: [" << tagPos.transpose() << "] mm\n";
    std::cout << "PASSED\n";
}

void testPropagateDeviation()
{
    std::cout << "\n========== Test 5: Propagate Deviation ==========\n";

    DeviationCorrector corrector;
    Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
    tFc(2, 3) = 150.0;
    corrector.setHandEyeCalibration(tFc);

    Eigen::Matrix4d tBaseTagStd = Eigen::Matrix4d::Identity();
    tBaseTagStd(0, 3) = 400.0;
    tBaseTagStd(1, 3) = 200.0;
    tBaseTagStd(2, 3) = 300.0;

    std::vector<Eigen::Matrix4d> rels;
    {
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        t(0, 3) =  100.0;
        rels.push_back(t);
    }
    {
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        t(0, 3) = -100.0;
        rels.push_back(t);
    }
    {
        Eigen::Matrix4d t = Eigen::Matrix4d::Identity();
        t(1, 3) =  100.0;
        rels.push_back(t);
    }

    Eigen::Matrix4d tBaseTagNew = tBaseTagStd;
    tBaseTagNew(0, 3) += 50.0;
    tBaseTagNew(1, 3) += 30.0;

    auto newPoses = corrector.propagateDeviation(tBaseTagNew, rels);
    for (size_t i = 0; i < newPoses.size(); ++i)
    {
        std::cout << "Point " << (i + 1) << ": X=" << newPoses[i].x
                  << " Y=" << newPoses[i].y << " Z=" << newPoses[i].z << "\n";
    }

    assert(std::abs(newPoses[0].x - 550.0) < 0.01);
    assert(std::abs(newPoses[0].y - 230.0) < 0.01);
    std::cout << "PASSED\n";
}

void testFullWorkflow()
{
    std::cout << "\n========== Test 6: Full Teach-Production Workflow ==========\n";

    MultiPointServo servo;
    Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
    tFc(2, 3) = 150.0;
    servo.setHandEyeCalibration(tFc);

    // ===== 示教阶段 =====
    std::cout << "\n--- Teaching ---\n";

    servo.startTeaching("TestRecipe");

    // 标准点: 龙门架在 (500, 300, 200)
    Pose6D stdPose(400.0, 200.0, 500.0, 0.0, -90.0, 0.0);
    TagDetection stdTag;
    stdTag.tvec = Eigen::Vector3d(0.0, 0.0, 0.5);
    stdTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);
    stdTag.id   = 0;

    servo.recordStandardPoint(stdPose, stdTag, 500.0f, 300.0f, 200.0f);
    std::cout << "Standard point recorded\n";

    // 拍照点1: 龙门架在 (520, 300, 180) — 和标准点不同!
    servo.addPhotoPoint("Front",  Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0),
                         520.0f, 300.0f, 180.0f);
    // 拍照点2: 龙门架在 (480, 350, 200)
    servo.addPhotoPoint("Side",   Pose6D(350.0, 250.0, 480.0, 0.0, -90.0, 0.0),
                         480.0f, 350.0f, 200.0f);
    // 拍照点3: 龙门架同标准点位置
    servo.addPhotoPoint("Top",    Pose6D(400.0, 200.0, 350.0, 0.0, -90.0, 0.0),
                         500.0f, 300.0f, 200.0f);

    servo.finishTeaching();
    std::cout << "Teaching done\n";

    // 保存配方
    servo.saveRecipe("test_recipe.json");
    std::cout << "Recipe saved\n";

    // ===== 生产阶段 =====
    std::cout << "\n--- Production ---\n";

    Pose6D prodPose(405.0, 203.0, 500.0, 0.0, -90.0, 0.0);
    TagDetection prodTag;
    prodTag.tvec = Eigen::Vector3d(0.01, 0.005, 0.5);
    prodTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.01);
    prodTag.id   = 0;

    auto newPoses = servo.computeNewPoses(prodPose, prodTag, 500.0f, 300.0f, 200.0f);
    for (const auto& [name, pose] : newPoses)
    {
        std::cout << "  " << name << ": X=" << pose.x << " Y=" << pose.y
                  << " Z=" << pose.z << " (标准基座系)\n";
    }
    assert(newPoses.size() == 3);
    // Front: 标准基座系 (475.0, 207.6, 470.0)
    assert(std::abs(newPoses[0].second.x - 475.0) < 0.01);
    assert(std::abs(newPoses[0].second.y - 207.6) < 0.01);
    assert(std::abs(newPoses[0].second.z - 470.0) < 0.02);
    // Side:  标准基座系 (335.0, 307.8, 489.0)
    assert(std::abs(newPoses[1].second.x - 335.0) < 0.01);
    assert(std::abs(newPoses[1].second.y - 307.8) < 0.01);
    assert(std::abs(newPoses[1].second.z - 489.0) < 0.01);
    // Top:   标准基座系 (405.0, 206.5, 360.0)
    assert(std::abs(newPoses[2].second.x - 405.0) < 0.01);
    assert(std::abs(newPoses[2].second.y - 206.5) < 0.01);
    assert(std::abs(newPoses[2].second.z - 360.0) < 0.01);

    // 龙门架调整 → 目标基座系
    auto adj1 = MultiPointServo::adjustForGantry(newPoses[0].second,
        500.0f, 300.0f, 200.0f, 520.0f, 300.0f, 180.0f);
    assert(std::abs(adj1.x - 455.0) < 0.01);
    assert(std::abs(adj1.y - 207.6) < 0.01);
    assert(std::abs(adj1.z - 490.0) < 0.02);

    auto adj2 = MultiPointServo::adjustForGantry(newPoses[1].second,
        500.0f, 300.0f, 200.0f, 480.0f, 350.0f, 200.0f);
    assert(std::abs(adj2.x - 355.0) < 0.01);
    assert(std::abs(adj2.y - 257.8) < 0.01);
    assert(std::abs(adj2.z - 489.0) < 0.01);

    auto adj3 = MultiPointServo::adjustForGantry(newPoses[2].second,
        500.0f, 300.0f, 200.0f, 500.0f, 300.0f, 200.0f);
    assert(std::abs(adj3.x - 405.0) < 0.01);
    assert(std::abs(adj3.y - 206.5) < 0.01);
    assert(std::abs(adj3.z - 360.0) < 0.01);

    std::cout << "PASSED\n";
}

void testGantryAdjust()
{
    std::cout << "\n========== Test 7: Gantry Frame Adjust ==========\n";

    // 标准基座系下的补偿位姿
    Pose6D poseStd(450.0, 200.0, 480.0, 0.0, -90.0, 0.0);

    // 标准点龙门架 (500, 300, 200), 拍照点龙门架 (520, 300, 180)
    Pose6D adjusted = MultiPointServo::adjustForGantry(
        poseStd,
        500.0f, 300.0f, 200.0f,   // 标准龙门架
        520.0f, 300.0f, 180.0f);  // 目标龙门架

    printPose("Standard frame", poseStd);
    printPose("Adjusted to target gantry", adjusted);

    // 龙门架X偏移+20, Z偏移-20 → 位姿X应-20, Z应+20
    assert(std::abs(adjusted.x - 430.0) < 0.01);
    assert(std::abs(adjusted.z - 500.0) < 0.01);
    std::cout << "PASSED\n";
}

void testGantryOnlyMovement()
{
    std::cout << "\n========== Test 8: Gantry-Only Movement (Production) ==========\n";

    MultiPointServo servo;
    Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
    tFc(2, 3) = 150.0;
    servo.setHandEyeCalibration(tFc);

    // 示教 (与 Test 6 相同)
    servo.startTeaching("GantryMoveTest");
    Pose6D stdPose(400.0, 200.0, 500.0, 0.0, -90.0, 0.0);
    TagDetection stdTag;
    stdTag.tvec = Eigen::Vector3d(0.0, 0.0, 0.5);
    stdTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);
    servo.recordStandardPoint(stdPose, stdTag, 500.0f, 300.0f, 200.0f);

    servo.addPhotoPoint("Front", Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0),
                         520.0f, 300.0f, 180.0f);
    servo.addPhotoPoint("Top", Pose6D(400.0, 200.0, 350.0, 0.0, -90.0, 0.0),
                         500.0f, 300.0f, 200.0f);
    servo.finishTeaching();

    // 生产: 机械臂同数值位姿, Tag 不变, 但龙门架移动了 +50mm (X方向)
    auto newPoses = servo.computeNewPoses(stdPose, stdTag, 550.0f, 300.0f, 200.0f);

    assert(newPoses.size() == 2);
    // Front 在标准基座系: 龙门架偏移+50 导致 Tag 位置偏移+50 → 法兰位姿 X+50
    assert(std::abs(newPoses[0].second.x - 520.0) < 0.01);
    assert(std::abs(newPoses[0].second.y - 200.0) < 0.01);
    assert(std::abs(newPoses[0].second.z - 460.0) < 0.01);
    // Top 在标准基座系: G_3=G_std, 龙门架偏移+50 → X+50
    assert(std::abs(newPoses[1].second.x - 450.0) < 0.01);
    assert(std::abs(newPoses[1].second.y - 200.0) < 0.01);
    assert(std::abs(newPoses[1].second.z - 350.0) < 0.01);

    // 龙门架调整
    auto adj1 = MultiPointServo::adjustForGantry(newPoses[0].second,
        500.0f, 300.0f, 200.0f, 520.0f, 300.0f, 180.0f);
    assert(std::abs(adj1.x - 500.0) < 0.01);
    assert(std::abs(adj1.y - 200.0) < 0.01);
    assert(std::abs(adj1.z - 480.0) < 0.01);

    auto adj2 = MultiPointServo::adjustForGantry(newPoses[1].second,
        500.0f, 300.0f, 200.0f, 500.0f, 300.0f, 200.0f);
    assert(std::abs(adj2.x - 450.0) < 0.01);
    assert(std::abs(adj2.y - 200.0) < 0.01);
    assert(std::abs(adj2.z - 350.0) < 0.01);

    std::cout << "PASSED\n";
}

void testRecipeLoadSave()
{
    std::cout << "\n========== Test 9: Recipe Load/Save Roundtrip ==========\n";

    MultiPointServo servo1;
    Eigen::Matrix4d tFc = Eigen::Matrix4d::Identity();
    tFc(2, 3) = 150.0;
    servo1.setHandEyeCalibration(tFc);

    servo1.startTeaching("LoadSaveTest");
    TagDetection stdTag;
    stdTag.tvec = Eigen::Vector3d(0.0, 0.0, 0.5);
    stdTag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);
    servo1.recordStandardPoint(
        Pose6D(400.0, 200.0, 500.0, 0.0, -90.0, 0.0),
        stdTag, 500.0f, 300.0f, 200.0f);
    servo1.addPhotoPoint("P1", Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0),
                          520.0f, 300.0f, 180.0f);
    servo1.addPhotoPoint("P2", Pose6D(350.0, 250.0, 480.0, 0.0, -90.0, 0.0),
                          480.0f, 350.0f, 200.0f);
    servo1.finishTeaching();
    servo1.saveRecipe("test_loadsave.json");

    // 加载
    MultiPointServo servo2;
    servo2.setHandEyeCalibration(tFc);
    servo2.loadRecipe("test_loadsave.json");

    const auto& r2 = servo2.getCurrentRecipe();
    assert(r2.name == "LoadSaveTest");
    assert(r2.stdGantryX == 500.0f);
    assert(r2.photoPoints.size() == 2);
    assert(r2.photoPoints[0].name == "P1");
    assert(r2.photoPoints[0].gantryX == 520.0f);

    std::cout << "Loaded " << r2.photoPoints.size() << " points\n";
    std::cout << "Point 0: " << r2.photoPoints[0].name
              << " gantry=(" << r2.photoPoints[0].gantryX << ", "
              << r2.photoPoints[0].gantryY << ", "
              << r2.photoPoints[0].gantryZ << ")\n";
    std::cout << "PASSED\n";
}

void testLoadHandEyeFormats()
{
    std::cout << "\n========== Test 10: Load HandEye Both Formats ==========\n";

    // 格式1: {"T": [[row],...]}
    {
        std::ofstream f("test_he_format1.json");
        f << "{\"T\":[[1,0,0,50],[0,1,0,0],[0,0,1,100],[0,0,0,1]]}\n";
        f.close();

        DeviationCorrector c;
        assert(c.loadHandEyeFromFile("test_he_format1.json"));
        auto m = c.getHandEyeCalibration();
        assert(std::abs(m(0, 3) - 50.0) < 0.001);
        std::cout << "Format 1 (T key) OK\n";
    }

    // 格式2: {"result": [flat array]}
    {
        std::ofstream f("test_he_format2.json");
        f << "{\"result\":[1,0,0,50,0,1,0,0,0,0,1,100,0,0,0,1]}\n";
        f.close();

        DeviationCorrector c;
        assert(c.loadHandEyeFromFile("test_he_format2.json"));
        auto m = c.getHandEyeCalibration();
        assert(std::abs(m(0, 3) - 50.0) < 0.001);
        std::cout << "Format 2 (result key) OK\n";
    }

    std::cout << "PASSED\n";
}

int main()
{
    std::cout << "========================================\n";
    std::cout << "  Deviation Corrector Unit Tests v2.0\n";
    std::cout << "========================================\n";

    try
    {
        testPoseMatrixRoundtrip();
        testRodrigues();
        testSingleCorrection();
        testComputeTagInBase();
        testPropagateDeviation();
        testFullWorkflow();
        testGantryAdjust();
        testGantryOnlyMovement();
        testRecipeLoadSave();
        testLoadHandEyeFormats();

        std::cout << "\n========================================\n";
        std::cout << "  All 10 Tests PASSED!\n";
        std::cout << "========================================\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "FAILED: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
