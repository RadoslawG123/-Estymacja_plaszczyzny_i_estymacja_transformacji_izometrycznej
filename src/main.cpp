#include "error_stats.hpp"
#include "io.hpp"
#include "plane_fitting.hpp"

#include <exception>
#include <iostream>

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <points.txt>\n";
        return 1;
    }

    try
    {
        const PointCloud cloud = loadPointCloud(argv[1]);
        std::cout << "Loaded " << cloud.size() << " points from " << cloud.name << '\n';

        const Plane plane = fitPlaneSVD(cloud);
        std::cout << "Plane: a=" << plane.a() << " b=" << plane.b()
                  << " c=" << plane.c() << " d=" << plane.d << '\n';

        const ErrorStats stats = computeErrorStats(planeDeviations(plane, cloud));
        std::cout << "RMS: " << stats.rms << "  max |dev|: " << stats.maxAbs << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
