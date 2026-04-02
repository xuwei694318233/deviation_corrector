/**
 * @file example_multi_point.cpp
 * @brief 多点位视觉伺服示例
 * 
 * 演示完整的示教-生产流程
 */

#include "../include/deviation_corrector.hpp"
#include <iostream>
#include <iomanip>

using namespace vision_servo;

// 模拟机器人控制器
class MockRobotController {
public:
    Pose6D getPosition() const { return current_pose_; }
    void moveTo(const Pose6D& pose) { 
        current_pose_ = pose;
        std::cout << "  [机器人] 移动到: X=" << pose.x << " Y=" << pose.y 
                  << " Z=" << pose.z << "\n";
    }
private:
    Pose6D current_pose_{500, 300, 400, 180, 0, 0};
};

// 模拟相机
class MockCamera {
public:
    void capture() { std::cout << "  [相机] 拍照完成\n"; }
};

// 模拟视觉检测器
class MockVisionDetector {
public:
    TagDetection detect(const std::string& scenario) {
        TagDetection tag;
        tag.id = 0;
        
        if (scenario == "teaching") {
            // 示教时的Tag位置
            tag.tvec = Eigen::Vector3d(0.0, 0.0, 0.5);  // 相机前方0.5m
            tag.rvec = Eigen::Vector3d(0.0, 0.0, 0.0);
        } else {
            // 生产时的Tag位置 (有偏移)
            tag.tvec = Eigen::Vector3d(0.005, 0.003, 0.5);  // 偏移5mm, 3mm
            tag.rvec = Eigen::Vector3d(0.0, 0.0, 0.01);     // 旋转约0.5度
        }
        
        return tag;
    }
};

int main() {
    std::cout << "========================================\n";
    std::cout << "  多点位视觉伺服示例\n";
    std::cout << "========================================\n\n";
    
    // 创建模拟设备
    MockRobotController robot;
    MockCamera camera;
    MockVisionDetector detector;
    
    // 创建多点位伺服控制器
    MultiPointServo servo;
    
    // 设置手眼标定
    Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
    T_flange_cam(2, 3) = 150.0;
    servo.getCorrector().setHandEyeCalibration(T_flange_cam);
    
    // ==================== 示教阶段 ====================
    std::cout << "【阶段1: 示教】\n";
    std::cout << "----------------------------------------\n";
    
    // 开始示教
    ServoRecipe recipe = servo.startTeaching("柔性拍摄配方_001");
    std::cout << "创建配方: " << recipe.name << "\n\n";
    
    // 移动到标准位置
    Pose6D std_pose(500, 300, 400, 180, 0, 0);
    robot.moveTo(std_pose);
    camera.capture();
    
    // 检测标准位置Tag
    TagDetection std_tag = detector.detect("teaching");
    std::cout << "  [视觉] 检测到Tag ID=" << std_tag.id 
              << " 位置=[" << std_tag.tvec.transpose() << "] m\n";
    
    // 记录标准点
    servo.recordStandardPoint(std_pose, std_tag);
    std::cout << "标准点已记录\n\n";
    
    // 示教多个拍照点
    std::vector<std::pair<std::string, Pose6D>> teaching_points = {
        {"拍照点1", Pose6D(550, 300, 380, 180, 0, 0)},
        {"拍照点2", Pose6D(450, 300, 380, 180, 0, 0)},
        {"拍照点3", Pose6D(500, 350, 380, 180, 0, 0)},
        {"拍照点4", Pose6D(500, 250, 380, 180, 0, 0)},
    };
    
    for (const auto& [name, pose] : teaching_points) {
        robot.moveTo(pose);
        camera.capture();
        int count = servo.addPhotoPoint(name, pose);
        std::cout << "  添加 " << name << " (共" << count << "个点位)\n\n";
    }
    
    // 完成示教
    recipe = servo.finishTeaching();
    std::cout << "示教完成! 共记录 " << recipe.photo_points.size() << " 个拍照点\n";
    
    // 保存配方
    servo.saveRecipe("workspace/paths/recipes/" + recipe.id + ".json");
    std::cout << "配方已保存\n\n";
    
    // ==================== 生产阶段 ====================
    std::cout << "【阶段2: 生产】\n";
    std::cout << "----------------------------------------\n";
    
    // 回到标准位置
    robot.moveTo(std_pose);
    camera.capture();
    
    // 检测生产环境下的Tag
    TagDetection prod_tag = detector.detect("production");
    std::cout << "  [视觉] 检测到Tag ID=" << prod_tag.id 
              << " 位置=[" << prod_tag.tvec.transpose() << "] m\n";
    
    // 获取当前机器人位姿
    Pose6D current_pose = robot.getPosition();
    
    // 计算偏差和所有点位的新位姿
    auto new_poses = servo.computeNewPoses(current_pose, prod_tag);
    
    std::cout << "\n偏差传播结果:\n";
    for (const auto& [name, pose] : new_poses) {
        std::cout << "  " << name << ": ";
        std::cout << "X=" << std::fixed << std::setprecision(2) << pose.x
                  << " Y=" << pose.y << " Z=" << pose.z << "\n";
    }
    
    // 执行纠偏后的运动
    std::cout << "\n执行纠偏运动:\n";
    for (const auto& [name, pose] : new_poses) {
        std::cout << "  移动到 " << name << "...\n";
        robot.moveTo(pose);
        camera.capture();
    }
    
    std::cout << "\n========================================\n";
    std::cout << "  生产流程完成!\n";
    std::cout << "========================================\n";
    
    return 0;
}
