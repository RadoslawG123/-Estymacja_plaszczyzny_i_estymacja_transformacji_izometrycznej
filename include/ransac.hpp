#pragma once

#include <cstddef>
#include <random>
#include <vector>

struct RansacParams
{
    double threshold = 0.05;  // max residual of an inlier
    int maxIterations = 1000; // upper limit, usually far fewer are needed
    double confidence = 0.99; // probability of drawing at least one clean sample
    unsigned seed = 42;       // fixed seed -> repeatable results
};

// What happened during RANSAC, used in reports.
struct RansacInfo
{
    bool used = false;
    RansacParams params;
    int iterations = 0;
    std::size_t inliers = 0;
};

namespace ransac
{

// Number of iterations needed to get an outlier-free sample of size
// sampleSize with the given confidence: log(1 - p) / log(1 - w^s).
int requiredIterations(double inlierRatio, int sampleSize, double confidence, int maxIterations);

// k different random indices from [0, n).
std::vector<std::size_t> randomSample(std::size_t n, std::size_t k, std::mt19937& rng);

std::vector<std::size_t> maskToIndices(const std::vector<bool>& mask);

} // namespace ransac
