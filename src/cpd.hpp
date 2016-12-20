#pragma once

#include "Eigen/Dense"

namespace hunky_dory {
Eigen::MatrixXd cpd_rigid(const Eigen::MatrixXd& fixed,
                          const Eigen::MatrixXd& moving, double sigma2);
Eigen::MatrixXd cpd_nonrigid(const Eigen::MatrixXd& fixed,
                             const Eigen::MatrixXd& moving, double sigma2);
}
