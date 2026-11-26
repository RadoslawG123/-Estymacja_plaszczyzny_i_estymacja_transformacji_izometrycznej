#pragma once

#include "error_stats.hpp"
#include "plane_fitting.hpp"
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
std::string currentDateTime();

} // namespace report

void writePlaneReport(std::ostream& out, const PointCloud& cloud, const Plane& plane,
                      const std::vector<double>& deviations);
