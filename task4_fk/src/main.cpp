#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

// DH Transform: For joint i (theta variable, others fixed)
Eigen::Matrix4d dhTransform(double theta, double d, double a, double alpha) {
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 0) = ct;    T(0, 1) = -st * ca; T(0, 2) =  st * sa;  T(0, 3) = a * ct;
    T(1, 0) = st;    T(1, 1) =  ct * ca; T(1, 2) = -ct * sa;  T(1, 3) = a * st;
    T(2, 0) = 0;     T(2, 1) =  sa;      T(2, 2) =  ca;       T(2, 3) = d;
    T(3, 0) = 0;     T(3, 1) =  0;       T(3, 2) =  0;        T(3, 3) = 1;
    return T;
}

// Forward Kinematics: Joint angles to end-effector pose
Eigen::Matrix4d forwardKinematics(const std::vector<double>& joint_angles_rad, 
                                  const std::vector<double>& dh_params) {  // dh_params flattened: a1,alpha1,d1,theta_offset1, a2,...
    size_t n_joints = joint_angles_rad.size();
    if (n_joints * 4 != dh_params.size()) {
        std::cerr << "Error: Mismatch between joints (" << n_joints << ") and DH params size (" << dh_params.size() << ")" << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix4d T_total = Eigen::Matrix4d::Identity();
    for (size_t i = 0; i < n_joints; ++i) {
        double theta = joint_angles_rad[i] + dh_params[i*4 + 3];  // Add theta offset if fixed
        double a = dh_params[i*4 + 0];
        double alpha = dh_params[i*4 + 1];
        double d = dh_params[i*4 + 2];
        Eigen::Matrix4d A = dhTransform(theta, d, a, alpha);
        T_total = T_total * A;
    }
    return T_total;
}

int main() {
    // Example DH params for a simple 3-DOF planar arm (adapt from doc, e.g., for 6DOF UR-like)
    // Format: [a1, alpha1, d1, theta_offset1, a2, alpha2, d2, theta_offset2, ...]
    // Sample: Link lengths a=1m, no twists/offsets
    std::vector<double> dh_params = {0.0, 0.0, 0.0, 0.0,   // Joint 1: Revolute base
                                     1.0, 0.0, 0.0, 0.0,   // Joint 2: Elbow
                                     1.0, 0.0, 0.0, 0.0};  // Joint 3: Wrist
    std::vector<double> joints_rad = {0.5, 0.3, 0.2};  // Sample joint angles (radians)

    Eigen::Matrix4d T = forwardKinematics(joints_rad, dh_params);

    // Extract position and orientation
    Eigen::Vector3d position = T.block<3,1>(0,3);
    Eigen::Matrix3d rotation = T.block<3,3>(0,0);
    Eigen::Quaterniond quat(rotation);

    std::cout << "Joints (rad): ";
    for (double j : joints_rad) std::cout << j << " ";
    std::cout << std::endl;
    std::cout << "Position: x=" << position.x() << ", y=" << position.y() << ", z=" << position.z() << std::endl;
    std::cout << "Quaternion: w=" << quat.w() << ", x=" << quat.x() << ", y=" << quat.y() << ", z=" << quat.z() << std::endl;

    // Home pose test (all zero)
    std::fill(joints_rad.begin(), joints_rad.end(), 0.0);
    T = forwardKinematics(joints_rad, dh_params);
    position = T.block<3,1>(0,3);
    std::cout << "Home Position: x=" << position.x() << ", y=" << position.y() << ", z=" << position.z() << std::endl;

    return 0;
}
