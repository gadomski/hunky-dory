#include <cpd/comparer.hpp>
#include <cpd/rigid.hpp>
#include <cpd/runner.hpp>

#include "cpd.hpp"

namespace hunky_dory {

Result cpd(const Matrix& source, const Matrix& target, const DocoptMap& args) {
    double sigma2 = std::stod(args.at("--sigma2").asString());
    double outliers = std::stod(args.at("--outliers").asString());

    cpd::Runner<cpd::Rigid, cpd::FgtComparer> runner;
    runner.sigma2(sigma2).outliers(outliers);
    cpd::Rigid::Result result = runner.run(source, target);

    std::cerr << "Runtime: " << result.runtime.count()
              << "s\nMotion:" << result.translation.transpose() << "\n";
    Result r;
    r.runtime = result.runtime.count();
    r.iterations = result.iterations;
    r.dx = result.translation(0);
    r.dy = result.translation(1);
    r.dz = result.translation(2);
    return r;
}
}
