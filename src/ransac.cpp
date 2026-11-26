#include "ransac.hpp"

#include <algorithm>
#include <cmath>

namespace ransac
{

int requiredIterations(double inlierRatio, int sampleSize, double confidence, int maxIterations)
{
    const double pGood = std::pow(inlierRatio, sampleSize);
    if (pGood <= 0.0)
        return maxIterations;
    if (pGood >= 1.0)
        return 1;

    const double n = std::log(1.0 - confidence) / std::log(1.0 - pGood);
    if (!std::isfinite(n) || n >= maxIterations)
        return maxIterations;
    return std::max(1, static_cast<int>(std::ceil(n)));
}

std::vector<std::size_t> randomSample(std::size_t n, std::size_t k, std::mt19937& rng)
{
    std::uniform_int_distribution<std::size_t> dist(0, n - 1);
    std::vector<std::size_t> sample;
    sample.reserve(k);
    while (sample.size() < k)
    {
        const std::size_t idx = dist(rng);
        if (std::find(sample.begin(), sample.end(), idx) == sample.end())
            sample.push_back(idx);
    }
    return sample;
}

std::vector<std::size_t> maskToIndices(const std::vector<bool>& mask)
{
    std::vector<std::size_t> idx;
    for (std::size_t i = 0; i < mask.size(); ++i)
        if (mask[i])
            idx.push_back(i);
    return idx;
}

} // namespace ransac
