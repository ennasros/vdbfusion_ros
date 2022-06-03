#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>

void PreProcessCloud(std::vector<Eigen::Vector3d>& points, float min_range, float max_range) {
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() > max_range; }),
        points.end());
    points.erase(
        std::remove_if(points.begin(), points.end(), [&](auto p) { return p.norm() < min_range; }),
        points.end());
}
