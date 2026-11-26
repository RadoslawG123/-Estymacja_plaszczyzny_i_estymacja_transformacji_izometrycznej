#include "report.hpp"

#include <cmath>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace report
{

namespace
{
const std::string kLine(72, '=');
const std::string kThinLine(72, '-');
} // namespace

void header(std::ostream& out, const std::string& title)
{
    out << kLine << '\n' << "  " << title << '\n' << kLine << '\n';
}

void section(std::ostream& out, const std::string& title)
{
    out << '\n' << kThinLine << '\n' << title << '\n' << kThinLine << '\n';
}

void statsBlock(std::ostream& out, const ErrorStats& stats)
{
    // mean of LSQ residuals is ~1e-17, don't print it as "-0.000000"
    const double mean = std::abs(stats.mean) < 1e-12 ? 0.0 : stats.mean;
    out << std::fixed << std::setprecision(6)
        << "  points      = " << stats.count << '\n'
        << "  RMS         = " << std::setw(14) << stats.rms << '\n'
        << "  mean        = " << std::setw(14) << mean << '\n'
        << "  mean |dev|  = " << std::setw(14) << stats.meanAbs << '\n'
        << "  max |dev|   = " << std::setw(14) << stats.maxAbs << '\n';
}

void ransacBlock(std::ostream& out, const RansacInfo& info, std::size_t totalPoints)
{
    const double pct = totalPoints ? 100.0 * static_cast<double>(info.inliers) /
                                         static_cast<double>(totalPoints)
                                   : 0.0;
    section(out, "RANSAC");
    out << std::fixed << std::setprecision(6)
        << "  threshold       = " << info.params.threshold << '\n'
        << "  confidence      = " << std::setprecision(3) << info.params.confidence << '\n'
        << "  max iterations  = " << info.params.maxIterations << '\n'
        << "  iterations done = " << info.iterations << '\n'
        << "  seed            = " << info.params.seed << '\n'
        << "  inliers         = " << info.inliers << " / " << totalPoints << " ("
        << std::setprecision(1) << pct << " %)\n"
        << "  outliers        = " << totalPoints - info.inliers << '\n';
}

std::string currentDateTime()
{
    const std::time_t now = std::time(nullptr);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

} // namespace report

void writePlaneReport(std::ostream& out, const PointCloud& cloud, const Plane& plane,
                      const std::vector<double>& deviations, const std::vector<bool>& inliers,
                      const RansacInfo& ransacInfo)
{
    const std::vector<bool> mask = inliers.empty() ? std::vector<bool>(cloud.size(), true) : inliers;

    report::header(out, "PLANE ESTIMATION REPORT");
    out << "Date:        " << report::currentDateTime() << '\n'
        << "Input file:  " << cloud.name << '\n'
        << "Points:      " << cloud.size() << '\n'
        << "Method:      "
        << (ransacInfo.used ? "RANSAC + least squares fit on inliers (SVD)"
                            : "least squares fit (SVD)")
        << '\n';

    report::section(out, "Estimated plane:  a*x + b*y + c*z + d = 0");
    out << std::fixed << std::setprecision(9)
        << "  a = " << std::setw(16) << plane.a() << '\n'
        << "  b = " << std::setw(16) << plane.b() << '\n'
        << "  c = " << std::setw(16) << plane.c() << '\n'
        << "  d = " << std::setw(16) << plane.d << '\n';

    if (ransacInfo.used)
    {
        report::ransacBlock(out, ransacInfo, cloud.size());
        report::section(out, "Errors - inliers only (point-to-plane distance)");
        report::statsBlock(out, computeErrorStats(deviations, mask));
        report::section(out, "Errors - all points");
        report::statsBlock(out, computeErrorStats(deviations));
    }
    else
    {
        report::section(out, "Errors (point-to-plane distance)");
        report::statsBlock(out, computeErrorStats(deviations));
    }

    report::section(out, "Deviations of individual points");
    out << std::setw(7) << "#" << std::setw(14) << "x" << std::setw(14) << "y"
        << std::setw(14) << "z" << std::setw(14) << "deviation";
    if (ransacInfo.used)
        out << "  status";
    out << '\n' << std::setprecision(6);

    for (std::size_t i = 0; i < cloud.size(); ++i)
    {
        const Point3& p = cloud[i];
        out << std::setw(7) << i + 1 << std::setw(14) << p.x() << std::setw(14) << p.y()
            << std::setw(14) << p.z() << std::setw(14) << deviations[i];
        if (ransacInfo.used)
            out << "  " << (mask[i] ? "inlier" : "OUTLIER");
        out << '\n';
    }
}
