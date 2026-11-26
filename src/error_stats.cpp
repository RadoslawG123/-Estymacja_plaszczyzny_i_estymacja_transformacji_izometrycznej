#include "error_stats.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

ErrorStats computeErrorStats(const std::vector<double>& values, const std::vector<bool>& mask)
{
    if (mask.size() != values.size())
        throw std::invalid_argument("computeErrorStats: mask size does not match");

    ErrorStats s;
    double sum = 0.0, sumAbs = 0.0, sumSq = 0.0;
    for (std::size_t i = 0; i < values.size(); ++i)
    {
        if (!mask[i])
            continue;
        const double v = values[i];
        sum += v;
        sumAbs += std::abs(v);
        sumSq += v * v;
        s.maxAbs = std::max(s.maxAbs, std::abs(v));
        ++s.count;
    }

    if (s.count > 0)
    {
        const double n = static_cast<double>(s.count);
        s.mean = sum / n;
        s.meanAbs = sumAbs / n;
        s.rms = std::sqrt(sumSq / n);
    }
    return s;
}

ErrorStats computeErrorStats(const std::vector<double>& values)
{
    return computeErrorStats(values, std::vector<bool>(values.size(), true));
}
