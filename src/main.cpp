#include "io.hpp"

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
        std::cout << "Centroid: " << cloud.centroid().transpose() << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
