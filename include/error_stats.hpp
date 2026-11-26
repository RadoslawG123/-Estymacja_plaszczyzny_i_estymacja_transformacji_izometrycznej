#pragma once

#include <cstddef>
#include <vector>

// Basic statistics of residuals (deviations).
struct ErrorStats
{
    std::size_t count = 0;
    double rms = 0.0;     // sqrt(sum(v^2) / n)
    double mean = 0.0;    // mean of v
    double meanAbs = 0.0; // mean of |v|
    double maxAbs = 0.0;  // max of |v|
};

ErrorStats computeErrorStats(const std::vector<double>& values);

// Only values where mask[i] == true are taken into account.
ErrorStats computeErrorStats(const std::vector<double>& values, const std::vector<bool>& mask);
