#include <iostream>
#include <cmath>
#include <iomanip>
#include <sstream> 
#include <cstdlib> 
#include <vector>
#include "roarm_moveit_cmd/solver.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "roarm_msgs/srv/get_pose_cmd.hpp"
#include "roarm_msgs/srv/move_joint_cmd.hpp"
#include "roarm_msgs/srv/move_line_cmd.hpp"
#include "roarm_msgs/srv/move_circle_cmd.hpp"

struct Pose {
    double x, y, z;
    double roll, pitch;
};

std::string get_roarm_model() {
    const char* env_val = std::getenv("ROARM_MODEL");
    if (env_val == nullptr) {
        std::cerr << "no ROARM_MODEL!" << std::endl;
        return "";  
    }
    return std::string(env_val); 
}

std::vector<Pose> generateLinearTrajectory(const Pose& startPose, const std::vector<double>& endPoint, int numPoints) {
    std::vector<Pose> trajectory;

    double x0 = startPose.x;
    double y0 = startPose.y;
    double z0 = startPose.z;
    double roll = startPose.roll;
    double pitch = startPose.pitch;

    double x1 = endPoint[0];
    double y1 = endPoint[1];
    double z1 = endPoint[2];

    for (int i = 0; i < numPoints; ++i) {
        double t = (numPoints > 1) ? static_cast<double>(i) / (numPoints - 1) : 0.0;

        double x = x0 + t * (x1 - x0);
        double y = y0 + t * (y1 - y0);
        double z = z0 + t * (z1 - z0);

        trajectory.push_back({x, y, z, roll, pitch});
    }

    return trajectory;
}

void crossProduct(double ax, double ay, double az,
                  double bx, double by, double bz,
                  double& rx, double& ry, double& rz) {
    rx = ay * bz - az * by;
    ry = az * bx - ax * bz;
    rz = ax * by - ay * bx;
}

void normalize(double& x, double& y, double& z) {
    double length = std::sqrt(x*x + y*y + z*z);
    if (length < 1e-6) return;
    x /= length;
    y /= length;
    z /= length;
}

std::vector<Pose> generateCircularTrajectory(
    const Pose& startPose, 
    const std::vector<double>& viaPoint,
    const std::vector<double>& endPoint,
    int numPoints) 
{
    std::vector<Pose> trajectory;
    if (numPoints < 2) return trajectory;

    const double x0 = startPose.x;
    const double y0 = startPose.y;
    const double z0 = startPose.z;
    const double roll = startPose.roll;
    const double pitch = startPose.pitch;

    const double x1 = viaPoint[0];
    const double y1 = viaPoint[1];
    const double z1 = viaPoint[2];

    const double x2 = endPoint[0];
    const double y2 = endPoint[1];
    const double z2 = endPoint[2];

    double vec1X = x1 - x0;
    double vec1Y = y1 - y0;
    double vec1Z = z1 - z0;

    double vec2X = x2 - x0;
    double vec2Y = y2 - y0;
    double vec2Z = z2 - z0;

    double normalX, normalY, normalZ;
    crossProduct(vec1X, vec1Y, vec1Z, vec2X, vec2Y, vec2Z, normalX, normalY, normalZ);
    normalize(normalX, normalY, normalZ);

    double mid1X = (x0 + x1) / 2;
    double mid1Y = (y0 + y1) / 2;
    double mid1Z = (z0 + z1) / 2;

    double mid2X = (x1 + x2) / 2;
    double mid2Y = (y1 + y2) / 2;
    double mid2Z = (z1 + z2) / 2;

    double dir1X = x1 - x0;
    double dir1Y = y1 - y0;
    double dir1Z = z1 - z0;

    double dir2X = x2 - x1;
    double dir2Y = y2 - y1;
    double dir2Z = z2 - z1;

    double perp1X, perp1Y, perp1Z;
    crossProduct(dir1X, dir1Y, dir1Z, normalX, normalY, normalZ, perp1X, perp1Y, perp1Z);
    normalize(perp1X, perp1Y, perp1Z);

    double perp2X, perp2Y, perp2Z;
    crossProduct(dir2X, dir2Y, dir2Z, normalX, normalY, normalZ, perp2X, perp2Y, perp2Z);
    normalize(perp2X, perp2Y, perp2Z);

    double A = perp1X, B = -perp2X;
    double C = perp1Y, D = -perp2Y;
    double E = mid2X - mid1X;
    double F = mid2Y - mid1Y;

    double det = A * D - B * C;
    if (std::abs(det) < 1e-6) {
        std::cerr << "error" << std::endl;
        return trajectory;
    }

    double t = (D * E - B * F) / det;
    double centerX = mid1X + t * perp1X;
    double centerY = mid1Y + t * perp1Y;
    double centerZ = mid1Z + t * perp1Z;

    double radius = std::sqrt((x0 - centerX)*(x0 - centerX) +
                              (y0 - centerY)*(y0 - centerY) +
                              (z0 - centerZ)*(z0 - centerZ));

    double startVecX = x0 - centerX;
    double startVecY = y0 - centerY;
    double startVecZ = z0 - centerZ;

    double endVecX = x2 - centerX;
    double endVecY = y2 - centerY;
    double endVecZ = z2 - centerZ;

    double uX = startVecX;
    double uY = startVecY;
    double uZ = startVecZ;
    normalize(uX, uY, uZ);

    double vX, vY, vZ;
    crossProduct(normalX, normalY, normalZ, uX, uY, uZ, vX, vY, vZ);

    double startAngle = std::atan2(vX * startVecX + vY * startVecY + vZ * startVecZ,
                                   uX * startVecX + uY * startVecY + uZ * startVecZ);
    double endAngle = std::atan2(vX * endVecX + vY * endVecY + vZ * endVecZ,
                                 uX * endVecX + uY * endVecY + uZ * endVecZ);

    if (endAngle < startAngle) {
        endAngle += 2 * M_PI;
    }

    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / (numPoints - 1);
        double angle = startAngle + t * (endAngle - startAngle);

        double x = centerX + radius * (std::cos(angle) * uX + std::sin(angle) * vX);
        double y = centerY + radius * (std::cos(angle) * uY + std::sin(angle) * vY);
        double z = centerZ + radius * (std::cos(angle) * uZ + std::sin(angle) * vZ);

        trajectory.push_back({x, y, z, roll, pitch});
    }

    return trajectory;
}


