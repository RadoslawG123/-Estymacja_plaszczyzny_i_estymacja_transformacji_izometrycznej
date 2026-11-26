#pragma once

#include "types.hpp"

#include <cstddef>
#include <optional>
#include <vector>

// Plane in the form a*x + b*y + c*z + d = 0, where (a, b, c) is a unit normal.
struct Plane
{
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    double d = 0.0;

    double a() const { return normal.x(); }
    double b() const { return normal.y(); }
    double c() const { return normal.z(); }

    // Signed distance of a point from the plane (positive on the normal side).
    double signedDistance(const Point3& p) const { return normal.dot(p) + d; }

    // Plane through three points, empty if they are (almost) collinear.
    static std::optional<Plane> throughPoints(const Point3& p1, const Point3& p2, const Point3& p3);
};

// Least squares plane fit: centroid + SVD of the centered points.
// The normal is the right singular vector with the smallest singular value.
// Throws std::invalid_argument for less than 3 points.
Plane fitPlaneSVD(const PointCloud& cloud);

// Same, but only the points with the given indices are used.
Plane fitPlaneSVD(const PointCloud& cloud, const std::vector<std::size_t>& indices);
