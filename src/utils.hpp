#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <entwine/types/bbox.hpp>
#include <entwine/types/schema.hpp>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>

#include "docopt.h"

namespace hunky_dory {
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    Matrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vector;
typedef std::map<std::string, docopt::value> DocoptMap;

pdal::Stage* infer_and_create_reader(pdal::StageFactory&, const std::string&);
Matrix point_view_to_matrix(const pdal::PointViewPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(const Matrix&);
Matrix pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZ>&);
entwine::BBox box2d_to_bbox(const pdal::BOX2D&);

struct CroppedFile {
    CroppedFile(const std::string&, const pdal::BOX2D&);
    Matrix matrix;
    double time;
};

struct Result {
    double runtime;
    int iterations;
    double dx;
    double dy;
    double dz;
};

static const entwine::Schema ENTWINE_XYZ_SCHEMA(
    {entwine::DimInfo("X", "floating", 8), entwine::DimInfo("Y", "floating", 8),
     entwine::DimInfo("Z", "floating", 8)});
}
