#include <pcl/registration/icp.h>

#include "utils.hpp"

namespace hunky_dory {
Result icp(const Matrix& source, const Matrix& target, const DocoptMap& args) {
    auto tic = std::chrono::high_resolution_clock::now();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    auto s = eigen_to_pcl(source);
    auto t = eigen_to_pcl(target);
    // yes source and target are backwards, deal with it
    icp.setInputSource(t);
    icp.setInputTarget(s);
    pcl::PointCloud<pcl::PointXYZ> f;
    icp.align(f);
    std::cerr << "has converged:" << icp.hasConverged()
              << " score: " << icp.getFitnessScore() << std::endl;
    Matrix final_ = pcl_to_eigen(f);
    Vector translation = (target - final_).colwise().mean();
    std::cerr << "Motion: " << translation.transpose() << std::endl;
    Result r;
    auto toc = std::chrono::high_resolution_clock::now();
    r.runtime =
        std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic)
            .count();
    r.iterations = 0;
    r.dx = translation(0);
    r.dy = translation(1);
    r.dz = translation(2);
    return r;
}
}
