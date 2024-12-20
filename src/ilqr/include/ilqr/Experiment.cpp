#include "./Experiment.h"
bool isCollision(const ilqr::Vehicle &v1, const ilqr::Vehicle &v2)
{
    // Helper lambda to rotate points
    auto rotatePoint = [](double x, double y, double yaw)
    {
        double xr = x * cos(yaw) - y * sin(yaw);
        double yr = x * sin(yaw) + y * cos(yaw);
        return std::make_pair(xr, yr);
    };

    // Get vehicle corners
    auto getCorners = [&](const ilqr::Vehicle &v)
    {
        double half_width = v.width / 2.0;
        double half_length = v.length / 2.0;
        std::vector<std::pair<double, double>> corners;
        corners.push_back(rotatePoint(-half_length, -half_width, v.yaw));
        corners.push_back(rotatePoint(half_length, -half_width, v.yaw));
        corners.push_back(rotatePoint(half_length, half_width, v.yaw));
        corners.push_back(rotatePoint(-half_length, half_width, v.yaw));
        for (auto &corner : corners)
        {
            corner.first += v.x;
            corner.second += v.y;
        }
        return corners;
    };

    std::vector<std::pair<double, double>> corners1 = getCorners(v1);
    std::vector<std::pair<double, double>> corners2 = getCorners(v2);

    // Check for separating axis theorem
    auto axes = {
        atan2(corners1[1].second - corners1[0].second, corners1[1].first - corners1[0].first),
        atan2(corners1[3].second - corners1[0].second, corners1[3].first - corners1[0].first),
        atan2(corners2[1].second - corners2[0].second, corners2[1].first - corners2[0].first),
        atan2(corners2[3].second - corners2[0].second, corners2[3].first - corners2[0].first)
    };

    for (auto axis : axes)
    {
        double min1 = std::numeric_limits<double>::infinity();
        double max1 = -std::numeric_limits<double>::infinity();
        double min2 = std::numeric_limits<double>::infinity();
        double max2 = -std::numeric_limits<double>::infinity();

        for (auto corner : corners1)
        {
            double projection = corner.first * cos(axis) + corner.second * sin(axis);
            min1 = std::min(min1, projection);
            max1 = std::max(max1, projection);
        }

        for (auto corner : corners2)
        {
            double projection = corner.first * cos(axis) + corner.second * sin(axis);
            min2 = std::min(min2, projection);
            max2 = std::max(max2, projection);
        }

        if (max1 < min2 || max2 < min1)
        {
            return false; // No collision
        }
    }

    return true; // Collision detected
}