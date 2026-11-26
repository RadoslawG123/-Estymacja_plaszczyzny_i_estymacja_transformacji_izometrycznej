// Generates synthetic test data with known ground truth.
// Usage: generate_data [output_dir]   (default: data)

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>

namespace
{

std::mt19937 rng(2024);

double uniform(double lo, double hi)
{
    return std::uniform_real_distribution<double>(lo, hi)(rng);
}

double gauss(double sigma)
{
    return std::normal_distribution<double>(0.0, sigma)(rng);
}

std::ofstream openFile(const std::string& path)
{
    std::ofstream f(path);
    if (!f)
    {
        std::cerr << "cannot write " << path << '\n';
        std::exit(1);
    }
    f << std::fixed << std::setprecision(6);
    return f;
}

// Points on the plane n.x + d = 0, inside a 10 x 10 square around the
// point closest to the origin, with gaussian noise along the normal.
// outlierRatio of the points is replaced by random points in a box.
void writePlane(const std::string& path, std::size_t n, double noise, double outlierRatio)
{
    const Eigen::Vector3d normal = Eigen::Vector3d(0.3, -0.2, 1.0).normalized();
    const double d = -2.0;
    const Eigen::Vector3d origin = -d * normal;
    const Eigen::Vector3d e1 = normal.unitOrthogonal();
    const Eigen::Vector3d e2 = normal.cross(e1);

    std::ofstream f = openFile(path);
    f << "# synthetic plane, ground truth (a b c d): " << normal.x() << ' ' << normal.y() << ' '
      << normal.z() << ' ' << d << '\n'
      << std::defaultfloat << "# points: " << n << ", noise sigma: " << noise
      << ", outliers: " << outlierRatio * 100 << " %\n"
      << std::fixed;

    for (std::size_t i = 0; i < n; ++i)
    {
        Eigen::Vector3d p;
        if (uniform(0.0, 1.0) < outlierRatio)
            p = origin + Eigen::Vector3d(uniform(-5, 5), uniform(-5, 5), uniform(-4, 4));
        else
            p = origin + uniform(-5, 5) * e1 + uniform(-5, 5) * e2 + gauss(noise) * normal;
        f << p.x() << ' ' << p.y() << ' ' << p.z() << '\n';
    }
    std::cout << "written " << path << '\n';
}

} // namespace

int main(int argc, char* argv[])
{
    const std::string dir = argc > 1 ? argv[1] : "data";

    writePlane(dir + "/plane.txt", 300, 0.01, 0.0);
    writePlane(dir + "/plane_outliers.txt", 300, 0.01, 0.25);
    return 0;
}
