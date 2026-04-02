/**
 * @file example_single_correction.cpp
 * @brief 单点纠偏示例
 * 
 * 演示如何使用 DeviationCorrector 进行单点纠偏
 */

#include "../include/deviation_corrector.hpp"
#include <iostream>
#include <iomanip>

using namespace vision_servo;

int main() {
    std::cout << "========================================\n";
    std::cout << "  单点纠偏示例\n";
    std::cout << "========================================\n\n";
    
    // 1. 创建纠偏器
    DeviationCorrector corrector;
    
    // 2. 设置手眼标定矩阵
    // 实际应用中应从标定文件加载
    Eigen::Matrix4d T_flange_cam = Eigen::Matrix4d::Identity();
    T_flange_cam(0, 3) = 0.0;     // X偏移
    T_flange_cam(1, 3) = 0.0;     // Y偏移
    T_flange_cam(2, 3) = 150.0;   // Z偏移 (相机距离法兰150mm)
    
    // 设置旋转 (示例: 相机相对于法兰有微小倾斜)
    double tilt_angle = 5.0 * 3.14159 / 180.0;  // 5度倾斜
    T_flange_cam.block<3,3>(0,0) = DeviationCorrector::eulerXYZToMatrix(
        Eigen::Vector3d(tilt_angle, 0, 0), false);
    
    corrector.setHandEyeCalibration(T_flange_cam);
    std::cout << "手眼标定矩阵已设置\n\n";
    
    // 3. 获取当前机械臂位姿 (实际应用中从机器人控制器读取)
    Pose6D current_pose;
    current_pose.x = 500.0;   // mm
    current_pose.y = 300.0;
    current_pose.z = 400.0;
    current_pose.rx = 180.0;  // deg (翻转)
    current_pose.ry = 0.0;
    current_pose.rz = 0.0;
    
    std::cout << "当前位姿:\n";
    std::cout << "  位置: X=" << current_pose.x << " Y=" << current_pose.y 
              << " Z=" << current_pose.z << " mm\n";
    std::cout << "  姿态: RX=" << current_pose.rx << " RY=" << current_pose.ry 
              << " RZ=" << current_pose.rz << " deg\n\n";
    
    // 4. 视觉检测到的偏差 (实际应用中从视觉系统获取)
    // 场景: 检测到物体向左偏移了30mm，需要机械臂向左追
    DeviationResult deviation;
    deviation.dx = -30.0;   // 物体在相机X方向左移30mm → 相机需要左移
    deviation.dy = 10.0;    // 物体在相机Y方向上移10mm
    deviation.dz = 0.0;
    deviation.drx = 0.0;
    deviation.dry = 0.0;
    deviation.drz = 2.0;    // 物体旋转了2度
    
    std::cout << "视觉偏差:\n";
    std::cout << "  位置偏差: dX=" << deviation.dx << " dY=" << deviation.dy 
              << " dZ=" << deviation.dz << " mm\n";
    std::cout << "  旋转偏差: dRX=" << deviation.drx << " dRY=" << deviation.dry 
              << " dRZ=" << deviation.drz << " deg\n\n";
    
    // 5. 计算纠偏后的目标位姿
    try {
        Pose6D target_pose = corrector.calculateCorrection(current_pose, deviation);
        
        std::cout << "纠偏后目标位姿:\n";
        std::cout << "  位置: X=" << std::fixed << std::setprecision(3)
                  << target_pose.x << " Y=" << target_pose.y 
                  << " Z=" << target_pose.z << " mm\n";
        std::cout << "  姿态: RX=" << target_pose.rx << " RY=" << target_pose.ry 
                  << " RZ=" << target_pose.rz << " deg\n\n";
        
        // 6. 计算变化量
        std::cout << "位姿变化量:\n";
        std::cout << "  位置: dX=" << (target_pose.x - current_pose.x)
                  << " dY=" << (target_pose.y - current_pose.y)
                  << " dZ=" << (target_pose.z - current_pose.z) << " mm\n";
        std::cout << "  姿态: dRX=" << (target_pose.rx - current_pose.rx)
                  << " dRY=" << (target_pose.ry - current_pose.ry)
                  << " dRZ=" << (target_pose.rz - current_pose.rz) << " deg\n\n";
        
        std::cout << "提示: 将上述目标位姿发送给机器人执行纠偏运动\n";
        
    } catch (const std::exception& e) {
        std::cerr << "纠偏计算失败: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
