#include "plane_fitting.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <cmath>
#include <numeric>
#include <stdexcept>

namespace
{

// The normal from SVD has an arbitrary sign. Flip it so that results are
// repeatable: the largest component of the normal is always positive.
void fixOrientation(Plane& plane)
{
    Eigen::Index maxIdx = 0;
    plane.normal.cwiseAbs().maxCoeff(&maxIdx);
    if (plane.normal(maxIdx) < 0.0)
    {
        plane.normal = -plane.normal;
        plane.d = -plane.d;
    }
}

} // namespace

std::optional<Plane> Plane::throughPoints(const Point3& p1, const Point3& p2, const Point3& p3)
{
    const Eigen::Vector3d n = (p2 - p1).cross(p3 - p1);
    const double len = n.norm();
    const double scale = (p2 - p1).norm() * (p3 - p1).norm();
    if (len < 1e-12 || len < 1e-9 * scale)
        return std::nullopt;

    Plane plane;
    plane.normal = n / len;
    plane.d = -plane.normal.dot(p1);
    fixOrientation(plane);
    return plane;
}

Plane fitPlaneSVD(const PointCloud& cloud, const std::vector<std::size_t>& indices)
{
    if (indices.size() < 3)
        throw std::invalid_argument("plane fitting needs at least 3 points");

    Point3 centroid = Point3::Zero();
    for (std::size_t i : indices)
        centroid += cloud[i];
    centroid /= static_cast<double>(indices.size());

    // N x 3 matrix of centered points
    Eigen::MatrixX3d A(static_cast<Eigen::Index>(indices.size()), 3);
    for (std::size_t k = 0; k < indices.size(); ++k)
        A.row(static_cast<Eigen::Index>(k)) = (cloud[indices[k]] - centroid).transpose();

    Eigen::JacobiSVD<Eigen::MatrixX3d> svd(A, Eigen::ComputeThinV);

    Plane plane;
    plane.normal = svd.matrixV().col(2).normalized();
    plane.d = -plane.normal.dot(centroid);
    fixOrientation(plane);
    return plane;
}

Plane fitPlaneSVD(const PointCloud& cloud)
{
    std::vector<std::size_t> all(cloud.size());
    std::iota(all.begin(), all.end(), std::size_t{0});
    return fitPlaneSVD(cloud, all);
}

std::vector<double> planeDeviations(const Plane& plane, const PointCloud& cloud)
{
    std::vector<double> dev;
    dev.reserve(cloud.size());
    for (const auto& p : cloud.points)
        dev.push_back(plane.signedDistance(p));
    return dev;
}
