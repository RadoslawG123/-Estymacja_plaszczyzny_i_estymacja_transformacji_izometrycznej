#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

using Point3 = Eigen::Vector3d;

// Set of 3D points loaded from a file.
struct PointCloud
{
    std::vector<Point3> points;
    std::string name; // usually the file the points were read from

    std::size_t size() const { return points.size(); }
    bool empty() const { return points.empty(); }

    const Point3& operator[](std::size_t i) const { return points[i]; }
    Point3& operator[](std::size_t i) { return points[i]; }

    // Points as columns of a 3xN matrix.
    Eigen::Matrix3Xd toMatrix() const
    {
        Eigen::Matrix3Xd m(3, static_cast<Eigen::Index>(points.size()));
        for (std::size_t i = 0; i < points.size(); ++i)
            m.col(static_cast<Eigen::Index>(i)) = points[i];
        return m;
    }

    Point3 centroid() const
    {
        Point3 c = Point3::Zero();
        for (const auto& p : points)
            c += p;
        return empty() ? c : Point3(c / static_cast<double>(points.size()));
    }
};
