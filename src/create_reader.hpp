#pragma once

#include <pdal/StageFactory.hpp>

namespace hunky_dory {
pdal::Stage* create_reader(pdal::StageFactory& factory,
                           const std::string& filename);
}
