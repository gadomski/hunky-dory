#include "cpd/nonrigid.hpp"
#include "cpd/rigid.hpp"

namespace hunky_dory {
Eigen::MatrixXd cpd_rigid(const Eigen::MatrixXd& fixed,
                          const Eigen::MatrixXd& moving, double sigma2) {
    cpd::Rigid rigid;
    rigid.sigma2(sigma2);
    return rigid.run(fixed, moving).points;
}

Eigen::MatrixXd cpd_nonrigid(const Eigen::MatrixXd& fixed,
                             const Eigen::MatrixXd& moving, double sigma2) {
    cpd::Nonrigid nonrigid;
    nonrigid.sigma2(sigma2);
    return nonrigid.run(fixed, moving).points;
}
}
