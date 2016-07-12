#include <cpd/rigid.hpp>

#include "cpd.hpp"

namespace hunky_dory {

Result cpd(const Matrix& source, const Matrix& target, const DocoptMap& args) {
    double sigma2 = std::stod(args.at("--sigma2").asString());
    double outlier = std::stod(args.at("--outlier").asString());

    ::cpd::Options options(std::cerr);
    options.set_sigma2(sigma2).set_outliers(outlier);
    ::cpd::RigidResult<Matrix> result = cpd::rigid(source, target, options);

    Vector translation = (target - result.moving).colwise().mean();
    std::cerr << "Runtime: " << result.runtime
              << "s\nMotion:" << translation.transpose() << "\n";
    Result r;
    r.runtime = result.runtime;
    r.iterations = result.iterations;
    r.dx = translation(0);
    r.dy = translation(1);
    r.dz = translation(2);
    return r;
}
}
