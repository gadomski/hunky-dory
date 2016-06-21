#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    Matrix;

pdal::Stage* infer_and_create_reader(pdal::StageFactory&, const std::string&);
Matrix point_view_to_matrix(const pdal::PointViewPtr);
pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(const Matrix&);
Matrix pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZ>&);

struct CroppedFile {
    CroppedFile(const std::string&, const pdal::BOX2D&);
    Matrix matrix;
    double time;
};
