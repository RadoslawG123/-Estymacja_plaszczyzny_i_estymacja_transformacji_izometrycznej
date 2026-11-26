#include "io.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace
{

std::string stripComment(const std::string& line)
{
    const auto pos = line.find('#');
    return pos == std::string::npos ? line : line.substr(0, pos);
}

bool isBlank(const std::string& s)
{
    return s.find_first_not_of(" \t\r") == std::string::npos;
}

} // namespace

PointCloud parsePointCloud(std::istream& in, const std::string& name)
{
    PointCloud cloud;
    cloud.name = name;

    std::string line;
    int lineNo = 0;
    while (std::getline(in, line))
    {
        ++lineNo;
        line = stripComment(line);
        if (isBlank(line))
            continue;

        for (char& c : line)
            if (c == ',' || c == ';')
                c = ' ';

        std::istringstream ss(line);
        double x, y, z;
        if (!(ss >> x >> y >> z))
            throw std::runtime_error(name + ":" + std::to_string(lineNo) +
                                     ": expected 3 numbers (x y z)");

        std::string rest;
        if (ss >> rest)
            throw std::runtime_error(name + ":" + std::to_string(lineNo) +
                                     ": unexpected value '" + rest + "'");

        cloud.points.emplace_back(x, y, z);
    }
    return cloud;
}

PointCloud loadPointCloud(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("cannot open file: " + path);
    return parsePointCloud(file, path);
}
