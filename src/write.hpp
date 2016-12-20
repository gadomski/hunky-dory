#pragma once

#include "Eigen/Dense"
#include "pdal/PointView.hpp"

namespace hunky_dory {
void write(const pdal::PointViewSet& pointViewSet,
           const std::vector<Eigen::MatrixXd>& results,
           const std::string& outfile, const std::string& driver,
           double edge_length, double radius, double no_data,
           double window_size);
}
