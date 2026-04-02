/**
 * @file test_deviation_corrector.cpp
 * @brief 纠偏算法动态库单元测试
 *
 * 此测试程序动态链接 deviation_corrector.dll 进行测试
 */

#include "deviation_corrector.hpp"
#include <iostream>
#include <iomanip>
#include <cassert>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

using namespace vision_servo;

// 辅助函数：打印位姿
void printPose(const std::string &label, const Pose6D &pose)
{
    std::cout << label << ":\n";
    std::cout << "  Position: X=" << std::fixed << std::setprecision(3)
              << pose.x << " Y=" << pose.y << " Z=" << pose.z << " mm\n";
    std::cout << "  Rotation: RX=" << pose.rx << " RY=" << pose.ry
              << " RZ=" << pose.rz << " deg\n";
}

// 辅助函数：打印矩阵
void printMatrix(const std::string &label, const Eigen::Matrix4d &matrix)
{
    std::cout << label << ":\n";
    for (int i = 0; i < 4; ++i)
    {
        std::cout << "  [";
        for (int j = 0; j < 4; ++j)
        {
            std::cout << std::fixed << std::setprecision(6) << std::setw(12) << matrix(i, j);
        }
        std::cout << " ]\n";
    }
}

// 测试库版本
void testLibraryVersion()
{
    std::cout << "\n========== Test 0: Library Version ==========\n";

    const char *version = deviation_corrector_version();
    std::cout << "Library Version: " << version << "\n";

    std::cout << "Test 0 PASSED!\n";
}

// 测试C接口
void testCInterface()
{
    std::cout << "\n========== Test C Interface ==========\n";

    // 创建纠偏器
    DeviationCorrector *corrector = deviation_corrector_create();
    assert(corrector != nullptr && "Failed to create corrector");
    std::cout << "Created corrector via C interface\n";

    // 设置手眼标定矩阵
    double hand_eye[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 150,
        0, 0, 0, 1};
    deviation_corrector_set_hand_eye(corrector, hand_eye);
    std::cout << "Set hand-eye calibration\n";

    // 计算纠偏
    double current_pose[6] = {500, 300, 400, 180, 0, 0};
    double deviation[6] = {-30, 10, 0, 0, 0, 2};
    double out_pose[6] = {0};

    deviation_corrector_calculate(corrector, current_pose, deviation, out_pose);

    std::cout << "Input pose: [" << current_pose[0] << ", " << current_pose[1]
              << ", " << current_pose[2] << ", " << current_pose[3]
              << ", " << current_pose[4] << ", " << current_pose[5] << "]\n";
    std::cout << "Deviation: [" << deviation[0] << ", " << deviation[1]
              << ", " << deviation[2] << ", " << deviation[3]
              << ", " << deviation[4] << ", " << deviation[5] << "]\n";
    std::cout << "Output pose: [" << out_pose[0] << ", " << out_pose[1]
              << ", " << out_pose[2] << ", " << out_pose[3]
              << ", " << out_pose[4] << ", " << out_pose[5] << "]\n";

    // 销毁纠偏器
    deviation_corrector_destroy(corrector);
    std::cout << "Destroyed corrector\n";

    std::cout << "Test C Interface PASSED!\n";
}

// 测试1：位姿与矩阵转换
void testPoseMatrixConversion()
{
    std::cout << "\n========== Test 1: Pose-Matrix Conversion ==========\n";

    Pose6D pose(100.0, 200.0, 300.0, 10.0, -20.0, 45.0);
    printPose("Original Pose", pose);

    Eigen::Matrix4d matrix = DeviationCorrector::poseToMatrix(pose, true);
    printMatrix("Pose to Matrix", matrix);

    Pose6D pose_back = DeviationCorrector::matrixToPose(matrix, true);
    printPose("Matrix to Pose", pose_back);

    // 验证往返转换误差
    double pos_error = std::abs(pose.x - pose_back.x) +
                       std::abs(pose.y - pose_back.y) +
                       std::abs(pose.z - pose_back.z);
    double rot_error = std::abs(pose.rx - pose_back.rx) +
                       std::abs(pose.ry - pose_back.ry) +
                       std::abs(pose.rz - pose_back.rz);

    std::cout << "Position error: " << pos_error << " mm\n";
    std::cout << "Rotation error: " << rot_error << " deg\n";

    assert(pos_error < 0.001 && "Position conversion error too large");
    assert(rot_error < 0.001 && "Rotation conversion error too large");

    std::cout << "Test 1 PASSED!\n";
}

// 测试2：Rodrigues变换
void testRodriguesConversion()
{
    std::cout << "\n========== Test 2: Rodrigues Conversion ==========\n";

    constexpr double PI = 3.14159265358979323846;

    // 测试绕Z轴旋转90度
    Eigen::Vector3d rvec(0, 0, PI / 2); // 绕Z轴旋转90度
    Eigen::Matrix3d R = DeviationCorrector::rodriguesToMatrix(rvec);

    std::cout << "Rotation vector: [0, 0, " << PI / 2 << "]\n";
    std::cout << "Rotation matrix:\n";
    std::cout << R << "\n";

    // 验证: R * [1, 0, 0]^T 应该约等于 [0, 1, 0]^T
    Eigen::Vector3d x_axis(1, 0, 0);
    Eigen::Vector3d rotated = R * x_axis;
    std::cout << "Rotated X-axis: [" << rotated.transpose() << "]\n";

    assert(std::abs(rotated(0)) < 0.001 && "Rodrigues conversion failed");
    assert(std::abs(rotated(1) - 1.0) < 0.001 && "Rodrigues conversion failed");

    std::cout << "Test 2 PASSED!\n";
}

// 测试3：单点纠偏
void testSinglePointCorrection()
{
    std::cout << "\n========== Test 3: Single Point Correction ==========\n";

    DeviationCorrector corrector;

    // 设置手眼标定矩阵 (示例)
    Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
    T_flange_cam(0, 3) = 50.0; // 相机在法兰X方向偏移50mm
    T_flange_cam(1, 3) = 0.0;
    T_flange_cam(2, 3) = 100.0; // 相机在法兰Z方向偏移100mm
    corrector.setHandEyeCalibration(T_flange_cam);

    // 当前位姿
    Pose6D current_pose(500.0, 300.0, 400.0, 0.0, -90.0, 0.0);
    printPose("Current Pose", current_pose);

    // 偏差 (物体向左偏了50mm)
    DeviationResult deviation = DeviationResult::xyPlane(-50.0, 0.0, 0.0);
    std::cout << "Deviation: dx=" << deviation.dx << " mm, dy=" << deviation.dy
              << " mm, drz=" << deviation.drz << " deg\n";

    // 计算纠偏后的目标位姿
    Pose6D target_pose = corrector.calculateCorrection(current_pose, deviation);
    printPose("Target Pose (Corrected)", target_pose);

    std::cout << "Test 3 PASSED!\n";
}

// 测试4：Tag位姿计算
void testComputeTagInBase()
{
    std::cout << "\n========== Test 4: Compute Tag in Base ==========\n";

    DeviationCorrector corrector;

    // 设置手眼标定矩阵
    Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
    T_flange_cam(0, 3) = 0.0;
    T_flange_cam(1, 3) = 0.0;
    T_flange_cam(2, 3) = 150.0; // 相机在法兰Z方向150mm
    corrector.setHandEyeCalibration(T_flange_cam);

    // 机械臂位姿
    Pose6D robot_pose(400.0, 200.0, 500.0, 0.0, -90.0, 0.0);
    printPose("Robot Pose", robot_pose);

    // Tag在相机系中的位姿
    Eigen::Vector3d tag_tvec(0.0, 0.0, 0.5); // Tag在相机前方0.5米
    Eigen::Vector3d tag_rvec(0.0, 0.0, 0.0); // 无旋转

    std::cout << "Tag in Camera: tvec=[" << tag_tvec.transpose() << "] m\n";

    // 计算Tag在基座系中的位姿
    Eigen::Matrix4d T_base_tag = corrector.computeTagInBase(robot_pose, tag_tvec, tag_rvec);
    printMatrix("Tag in Base Frame", T_base_tag);

    // 提取Tag位置
    Eigen::Vector3d tag_pos_base = T_base_tag.block<3, 1>(0, 3);
    std::cout << "Tag Position in Base: [" << tag_pos_base.transpose() << "] mm\n";

    std::cout << "Test 4 PASSED!\n";
}

// 测试5：多点位偏差传播
void testMultiPointPropagation()
{
    std::cout << "\n========== Test 5: Multi-Point Propagation ==========\n";

    DeviationCorrector corrector;

    // 设置手眼标定
    Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
    T_flange_cam(2, 3) = 150.0;
    corrector.setHandEyeCalibration(T_flange_cam);

    // 假设已有标准Tag在基座系中的位姿
    Eigen::Matrix4d T_base_tag_std = Eigen::Matrix4d::Identity();
    T_base_tag_std(0, 3) = 400.0;
    T_base_tag_std(1, 3) = 200.0;
    T_base_tag_std(2, 3) = 300.0;

    // 计算相对变换 (假设有3个拍照点)
    std::vector<Eigen::Matrix4d> rel_transforms;

    // 点位1: 相对于Tag偏移 [+100, 0, 0]
    Eigen::Matrix4d T1 = Eigen::Matrix4d::Identity();
    T1(0, 3) = 100.0;
    rel_transforms.push_back(T1);

    // 点位2: 相对于Tag偏移 [-100, 0, 0]
    Eigen::Matrix4d T2 = Eigen::Matrix4d::Identity();
    T2(0, 3) = -100.0;
    rel_transforms.push_back(T2);

    // 点位3: 相对于Tag偏移 [0, +100, 0]
    Eigen::Matrix4d T3 = Eigen::Matrix4d::Identity();
    T3(1, 3) = 100.0;
    rel_transforms.push_back(T3);

    // 新的Tag位姿 (假设Tag移动了 +50mm X, +30mm Y)
    Eigen::Matrix4d T_base_tag_new = T_base_tag_std;
    T_base_tag_new(0, 3) += 50.0;
    T_base_tag_new(1, 3) += 30.0;

    std::cout << "Original Tag position: [" << T_base_tag_std.block<3, 1>(0, 3).transpose() << "]\n";
    std::cout << "New Tag position: [" << T_base_tag_new.block<3, 1>(0, 3).transpose() << "]\n";

    // 偏差传播
    std::vector<Pose6D> new_poses = corrector.propagateDeviation(T_base_tag_new, rel_transforms);

    std::cout << "\nNew poses after propagation:\n";
    for (size_t i = 0; i < new_poses.size(); ++i)
    {
        std::cout << "Point " << (i + 1) << ": ";
        std::cout << "X=" << new_poses[i].x << " Y=" << new_poses[i].y
                  << " Z=" << new_poses[i].z << " mm\n";
    }

    // 验证: 所有点位应该同样偏移 +50mm X, +30mm Y
    assert(std::abs(new_poses[0].x - (500.0 + 50.0)) < 0.001);
    assert(std::abs(new_poses[0].y - (200.0 + 30.0)) < 0.001);

    std::cout << "Test 5 PASSED!\n";
}

// 测试6：完整示教-生产流程
void testFullTeachProductionWorkflow()
{
    std::cout << "\n========== Test 6: Full Teach-Production Workflow ==========\n";

    // 创建控制器
    MultiPointServo servo;

    // 设置手眼标定
    Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
    T_flange_cam(2, 3) = 150.0;
    servo.getCorrector().setHandEyeCalibration(T_flange_cam);

    // ==================== 示教阶段 ====================
    std::cout << "\n--- Teaching Phase ---\n";

    // 开始示教
    ServoRecipe recipe = servo.startTeaching("TestRecipe");
    std::cout << "Created recipe: " << recipe.name << "\n";

    // 记录标准点
    Pose6D std_pose(400.0, 200.0, 500.0, 0.0, -90.0, 0.0);
    TagDetection std_tag;
    std_tag.tvec = Eigen::Vector3d(0.0, 0.0, 0.5);
    std_tag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);
    std_tag.id = 0;

    bool success = servo.recordStandardPoint(std_pose, std_tag);
    std::cout << "Standard point recorded: " << (success ? "OK" : "FAILED") << "\n";

    // 添加拍照点
    servo.addPhotoPoint("Point1", Pose6D(450.0, 200.0, 480.0, 0.0, -90.0, 0.0));
    servo.addPhotoPoint("Point2", Pose6D(350.0, 200.0, 480.0, 0.0, -90.0, 0.0));
    servo.addPhotoPoint("Point3", Pose6D(400.0, 250.0, 480.0, 0.0, -90.0, 0.0));

    // 完成示教
    recipe = servo.finishTeaching();
    std::cout << "Teaching completed with " << recipe.photo_points.size() << " points\n";

    // 保存配方
    servo.saveRecipe("test_recipe.json");
    std::cout << "Recipe saved to test_recipe.json\n";

    // ==================== 生产阶段 ====================
    std::cout << "\n--- Production Phase ---\n";

    // 模拟: Tag位置发生了变化
    Pose6D prod_pose(405.0, 203.0, 500.0, 0.0, -90.0, 0.0); // 机械臂位置略有不同
    TagDetection prod_tag;
    prod_tag.tvec = Eigen::Vector3d(0.01, 0.005, 0.5); // Tag相对位置变化
    prod_tag.rvec = Eigen::Vector3d(0.0, 0.0, 0.01);
    prod_tag.id = 0;

    // 计算新位姿
    auto new_poses = servo.computeNewPoses(prod_pose, prod_tag);

    std::cout << "Computed new poses:\n";
    for (const auto &[name, pose] : new_poses)
    {
        std::cout << "  " << name << ": ";
        std::cout << "X=" << pose.x << " Y=" << pose.y << " Z=" << pose.z << "\n";
    }

    std::cout << "Test 6 PASSED!\n";
}

// 主函数
int main()
{
    std::cout << "========================================\n";
    std::cout << "  Deviation Corrector DLL Unit Tests\n";
    std::cout << "  Testing Dynamic Library Interface\n";
    std::cout << "========================================\n";

    try
    {
        testLibraryVersion();
        testCInterface();
        testPoseMatrixConversion();
        testRodriguesConversion();
        testSinglePointCorrection();
        testComputeTagInBase();
        testMultiPointPropagation();
        testFullTeachProductionWorkflow();

        std::cout << "\n========================================\n";
        std::cout << "  All Tests PASSED!\n";
        std::cout << "========================================\n";
    }
    catch (const std::exception &e)
    {
        std::cerr << "Test FAILED with error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
