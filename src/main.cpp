#include "error_stats.hpp"
#include "io.hpp"
#include "plane_fitting.hpp"
#include "report.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void printUsage(const char* prog)
{
    std::cerr << "Usage:\n"
              << "  " << prog << " plane <cloud.txt> [options]\n"
              << "\nOptions:\n"
              << "  -o, --output <file>   report file (default: report_plane.txt)\n";
}

struct Options
{
    std::vector<std::string> inputs;
    std::string output;
};

Options parseOptions(const std::vector<std::string>& args)
{
    Options opt;
    for (std::size_t i = 0; i < args.size(); ++i)
    {
        const std::string& a = args[i];
        if (a == "-o" || a == "--output")
        {
            if (i + 1 >= args.size())
                throw std::invalid_argument("missing value for " + a);
            opt.output = args[++i];
        }
        else if (!a.empty() && a[0] == '-')
            throw std::invalid_argument("unknown option: " + a);
        else
            opt.inputs.push_back(a);
    }
    return opt;
}

std::ofstream openReport(const std::string& path)
{
    std::ofstream out(path);
    if (!out)
        throw std::runtime_error("cannot write report: " + path);
    return out;
}

int runPlane(const Options& opt)
{
    if (opt.inputs.size() != 1)
        throw std::invalid_argument("'plane' expects exactly one input file");

    const PointCloud cloud = loadPointCloud(opt.inputs[0]);
    const Plane plane = fitPlaneSVD(cloud);
    const std::vector<double> dev = planeDeviations(plane, cloud);
    const ErrorStats stats = computeErrorStats(dev);

    const std::string reportPath = opt.output.empty() ? "report_plane.txt" : opt.output;
    std::ofstream out = openReport(reportPath);
    writePlaneReport(out, cloud, plane, dev);

    std::cout << "Points:  " << cloud.size() << '\n'
              << "Plane:   a = " << plane.a() << ", b = " << plane.b()
              << ", c = " << plane.c() << ", d = " << plane.d << '\n'
              << "RMS:     " << stats.rms << '\n'
              << "Report saved to " << reportPath << '\n';
    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        printUsage(argv[0]);
        return 1;
    }

    const std::string mode = argv[1];
    try
    {
        const Options opt = parseOptions(std::vector<std::string>(argv + 2, argv + argc));
        if (mode == "plane")
            return runPlane(opt);

        printUsage(argv[0]);
        return 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
}
