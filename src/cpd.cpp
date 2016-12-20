#include "cpd.hpp"
#include "cpd/nonrigid.hpp"
#include "cpd/rigid.hpp"
#include "cpd/runner.hpp"

namespace hunky_dory {
Eigen::MatrixXd cpd_rigid(const Eigen::MatrixXd& fixed,
                          const Eigen::MatrixXd& moving, double sigma2) {
    cpd::Rigid rigid;
    std::unique_ptr<cpd::Comparer> comparer = cpd::Comparer::from_name("fgt");
    cpd::Runner<cpd::Rigid> runner(rigid);
    runner.comparer(std::move(comparer)).sigma2(sigma2);
    return runner.run(fixed, moving).points;
}

Eigen::MatrixXd cpd_nonrigid(const Eigen::MatrixXd& fixed,
                             const Eigen::MatrixXd& moving, double sigma2) {
    cpd::Nonrigid nonrigid;
    std::unique_ptr<cpd::Comparer> comparer = cpd::Comparer::from_name("fgt");
    cpd::Runner<cpd::Nonrigid> runner(nonrigid);
    runner.comparer(std::move(comparer)).sigma2(sigma2);
    return runner.run(fixed, moving).points;
}
}
