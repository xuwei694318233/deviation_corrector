/**
 * @file example_multi_point.cpp
 * @brief 多点位视觉伺服 — 示教-生产完整示例
 */

#include "../include/deviation_corrector.hpp"

#include <iostream>
#include <iomanip>

using namespace vision_servo;

/**
 * @brief 模拟机器人控制器
 */
class MockRobotController
{
public:
    Pose6D GetPosition() const
    {
        return currentPose_;
    }

    void MoveTo(const Pose6D& pose)
    {
        currentPose_ = pose;
        std::cout << "  [Robot] MoveTo: X=" << pose.x
                  << " Y=" << pose.y << " Z=" << pose.z << "\n";
    }

private:
    Pose6D currentPose_{500.0, 300.0, 400.0, 180.0, 0.0, 0.0};
};

/**
 * @brief 模拟相机
 */
class MockCamera
{
public:
    void Capture()
    {
        std::cout << "  [Camera] Captured\n";
    }
};

/**
 * @brief 模拟视觉检测器
 */
class MockVisionDetector
{
public:
    TagDetection Detect(const std::string& scenario)
    {
        TagDetection tag;
        tag.id = 0;

        if (scenario == "teaching")
        {
            tag.tvec = Eigen::Vector3d(0.0, 0.0, 0.5);
            tag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);
        }
        else
        {
            tag.tvec = Eigen::Vector3d(0.005, 0.003, 0.5);
            tag.rvec = Eigen::Vector3d(0.0, 0.0, 0.01);
        }

        return tag;
    }
};

int main()
{
    std::cout << "========================================\n";
    std::cout << "  多点位视觉伺服示例\n";
    std::cout << "========================================\n\n";

    MockRobotController robot;
    MockCamera camera;
    MockVisionDetector detector;

    MultiPointServo servo;

    // 手眼标定: 相机在法兰下方 150mm
    Eigen::Matrix4d tFlangeCam = Eigen::Matrix4d::Identity();
    tFlangeCam(2, 3) = 150.0;
    servo.setHandEyeCalibration(tFlangeCam);

    // ========== 示教 ==========
    std::cout << "[Phase 1: Teaching]\n";
    std::cout << "----------------------------------------\n";

    servo.startTeaching("flex_recipe_001");

    const Pose6D stdPose(500.0, 300.0, 400.0, 180.0, 0.0, 0.0);
    robot.MoveTo(stdPose);
    camera.Capture();

    TagDetection stdTag = detector.Detect("teaching");
    std::cout << "  [Vision] Tag ID=" << stdTag.id
              << " pos=[" << stdTag.tvec.transpose() << "] m\n";

    servo.recordStandardPoint(stdPose, stdTag, 500.0f, 300.0f, 200.0f);
    std::cout << "  Standard point recorded\n\n";

    const std::vector<std::pair<std::string, Pose6D>> teachingPoints = {
        {"点1", Pose6D(550.0, 300.0, 380.0, 180.0, 0.0, 0.0)},
        {"点2", Pose6D(450.0, 300.0, 380.0, 180.0, 0.0, 0.0)},
        {"点3", Pose6D(500.0, 350.0, 380.0, 180.0, 0.0, 0.0)},
        {"点4", Pose6D(500.0, 250.0, 380.0, 180.0, 0.0, 0.0)},
    };

    for (const auto& [name, pose] : teachingPoints)
    {
        robot.MoveTo(pose);
        camera.Capture();
        const int count = servo.addPhotoPoint(name, pose, 500.0f, 300.0f, 200.0f);
        std::cout << "  Added " << name << " (total " << count << ")\n";
    }

    const ServoRecipe recipe = servo.finishTeaching();
    std::cout << "\nTeaching done, " << recipe.photoPoints.size() << " points\n";

    servo.saveRecipe("recipe_" + recipe.id + ".json");
    std::cout << "Recipe saved\n\n";

    // ========== 生产 ==========
    std::cout << "[Phase 2: Production]\n";
    std::cout << "----------------------------------------\n";

    robot.MoveTo(stdPose);
    camera.Capture();

    const TagDetection prodTag = detector.Detect("production");
    std::cout << "  [Vision] Tag ID=" << prodTag.id
              << " pos=[" << prodTag.tvec.transpose() << "] m\n";

    const Pose6D curPose = robot.GetPosition();
    const auto newPoses = servo.computeNewPoses(curPose, prodTag, 500.0f, 300.0f, 200.0f);

    std::cout << "\nCompensated poses:\n";
    for (const auto& [name, pose] : newPoses)
    {
        std::cout << "  " << name << ": X=" << std::fixed << std::setprecision(2)
                  << pose.x << " Y=" << pose.y << " Z=" << pose.z << "\n";
    }

    std::cout << "\nExecuting correction:\n";
    for (const auto& [name, pose] : newPoses)
    {
        std::cout << "  Moving to " << name << "...\n";
        robot.MoveTo(pose);
        camera.Capture();
    }

    std::cout << "\n========================================\n";
    std::cout << "  Production complete!\n";
    std::cout << "========================================\n";

    return 0;
}
