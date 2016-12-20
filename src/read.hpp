#pragma once

#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

namespace hunky_dory {

pdal::PointViewSet read(const std::string& filename, pdal::PointTable& table,
                        const std::vector<pdal::Bounds>& bounds);
}
