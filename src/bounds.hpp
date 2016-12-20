#pragma once

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

namespace hunky_dory {
std::vector<pdal::Bounds> bounds(const std::string& filename,
                                 pdal::point_count_t capacity);
}
