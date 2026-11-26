#include "error_stats.hpp"
#include "io.hpp"
#include "plane_fitting.hpp"
#include "ransac.hpp"
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
              << "  -o, --output <file>   report file (default: report_plane.txt)\n"
              << "  --ransac              robust estimation with RANSAC (data with outliers)\n"
              << "  --threshold <value>   RANSAC inlier threshold (default: 0.05)\n"
              << "  --iterations <n>      RANSAC max iterations (default: 1000)\n"
              << "  --confidence <p>      RANSAC confidence (default: 0.99)\n"
              << "  --seed <n>            random seed (default: 42)\n";
}

struct Options
{
    std::vector<std::string> inputs;
    std::string output;
    bool useRansac = false;
    RansacParams ransac;
};

Options parseOptions(const std::vector<std::string>& args)
{
    Options opt;
    for (std::size_t i = 0; i < args.size(); ++i)
    {
        const std::string& a = args[i];
        auto value = [&]() -> const std::string& {
            if (i + 1 >= args.size())
                throw std::invalid_argument("missing value for " + a);
            return args[++i];
        };

        if (a == "-o" || a == "--output")
            opt.output = value();
        else if (a == "--ransac")
            opt.useRansac = true;
        else if (a == "--threshold")
            opt.ransac.threshold = std::stod(value());
        else if (a == "--iterations")
            opt.ransac.maxIterations = std::stoi(value());
        else if (a == "--confidence")
            opt.ransac.confidence = std::stod(value());
        else if (a == "--seed")
            opt.ransac.seed = static_cast<unsigned>(std::stoul(value()));
        else if (!a.empty() && a[0] == '-')
            throw std::invalid_argument("unknown option: " + a);
        else
            opt.inputs.push_back(a);
    }

    if (opt.ransac.maxIterations < 1)
        throw std::invalid_argument("--iterations must be at least 1");
    if (opt.ransac.confidence <= 0.0 || opt.ransac.confidence >= 1.0)
        throw std::invalid_argument("--confidence must be in (0, 1)");
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

    Plane plane;
    std::vector<bool> inliers(cloud.size(), true);
    RansacInfo info;
    if (opt.useRansac)
    {
        const PlaneRansacResult r = fitPlaneRansac(cloud, opt.ransac);
        plane = r.plane;
        inliers = r.inliers;
        info = r.info;
    }
    else
    {
        plane = fitPlaneSVD(cloud);
    }

    const std::vector<double> dev = planeDeviations(plane, cloud);
    const ErrorStats stats = computeErrorStats(dev, inliers);

    const std::string reportPath = opt.output.empty() ? "report_plane.txt" : opt.output;
    std::ofstream out = openReport(reportPath);
    writePlaneReport(out, cloud, plane, dev, inliers, info);

    std::cout << "Points:  " << cloud.size() << '\n'
              << "Plane:   a = " << plane.a() << ", b = " << plane.b()
              << ", c = " << plane.c() << ", d = " << plane.d << '\n';
    if (info.used)
        std::cout << "Inliers: " << info.inliers << " (" << info.iterations << " iterations)\n";
    std::cout << "RMS:     " << stats.rms << (info.used ? " (inliers)" : "") << '\n'
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
