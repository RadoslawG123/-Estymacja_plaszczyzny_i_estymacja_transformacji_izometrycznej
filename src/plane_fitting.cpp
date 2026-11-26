#include "plane_fitting.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
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

namespace
{

std::size_t markInliers(const Plane& plane, const PointCloud& cloud, double threshold,
                        std::vector<bool>& mask)
{
    mask.assign(cloud.size(), false);
    std::size_t count = 0;
    for (std::size_t i = 0; i < cloud.size(); ++i)
    {
        if (std::abs(plane.signedDistance(cloud[i])) <= threshold)
        {
            mask[i] = true;
            ++count;
        }
    }
    return count;
}

} // namespace

PlaneRansacResult fitPlaneRansac(const PointCloud& cloud, const RansacParams& params)
{
    if (cloud.size() < 3)
        throw std::invalid_argument("plane fitting needs at least 3 points");
    if (params.threshold <= 0.0)
        throw std::invalid_argument("RANSAC threshold must be positive");

    std::mt19937 rng(params.seed);
    PlaneRansacResult result;
    result.info.used = true;
    result.info.params = params;

    std::size_t bestCount = 0;
    std::vector<bool> mask;
    int needed = params.maxIterations;
    int it = 0;
    for (; it < needed; ++it)
    {
        const auto s = ransac::randomSample(cloud.size(), 3, rng);
        const auto candidate = Plane::throughPoints(cloud[s[0]], cloud[s[1]], cloud[s[2]]);
        if (!candidate)
            continue;

        const std::size_t count = markInliers(*candidate, cloud, params.threshold, mask);
        if (count > bestCount)
        {
            bestCount = count;
            result.plane = *candidate;
            result.inliers = mask;
            const double ratio = static_cast<double>(count) / static_cast<double>(cloud.size());
            needed = ransac::requiredIterations(ratio, 3, params.confidence, params.maxIterations);
        }
    }
    result.info.iterations = it;

    if (bestCount < 3)
        throw std::runtime_error("RANSAC could not find a plane, try a larger threshold");

    // Refine on all inliers and update the inlier set for the refined plane.
    result.plane = fitPlaneSVD(cloud, ransac::maskToIndices(result.inliers));
    if (markInliers(result.plane, cloud, params.threshold, mask) >= 3)
        result.inliers = mask;
    result.info.inliers = static_cast<std::size_t>(
        std::count(result.inliers.begin(), result.inliers.end(), true));
    return result;
}
