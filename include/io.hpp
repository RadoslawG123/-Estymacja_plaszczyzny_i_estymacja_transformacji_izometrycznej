#pragma once

#include "types.hpp"

#include <istream>
#include <string>

// Input format: one point per line, "x y z".
// Values can be separated by spaces, tabs, commas or semicolons.
// Empty lines and everything after '#' are ignored.
// Throws std::runtime_error if the file can't be opened or a line is malformed.
PointCloud loadPointCloud(const std::string& path);
PointCloud parsePointCloud(std::istream& in, const std::string& name = "");
