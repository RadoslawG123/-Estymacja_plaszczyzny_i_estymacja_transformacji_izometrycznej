#pragma once

#include "error_stats.hpp"
#include "plane_fitting.hpp"
#include "ransac.hpp"
#include "types.hpp"

#include <ostream>
#include <string>
#include <vector>

namespace report
{

// Small helpers for consistent formatting of all reports.
void header(std::ostream& out, const std::string& title);
void section(std::ostream& out, const std::string& title);
void statsBlock(std::ostream& out, const ErrorStats& stats);
void ransacBlock(std::ostream& out, const RansacInfo& info, std::size_t totalPoints);
std::string currentDateTime();

} // namespace report

// inliers: empty or all true when RANSAC was not used.
void writePlaneReport(std::ostream& out, const PointCloud& cloud, const Plane& plane,
                      const std::vector<double>& deviations, const std::vector<bool>& inliers,
                      const RansacInfo& ransacInfo);
