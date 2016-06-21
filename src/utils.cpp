#include <pdal/CropFilter.hpp>

#include "utils.hpp"

std::string infer_reader_driver(const pdal::StageFactory& factory,
                                const std::string& path) {
    std::string driver = factory.inferReaderDriver(path);
    if (driver.empty()) {
        std::stringstream ss;
        ss << "Unable to infer reader driver for path: " << path;
        throw std::runtime_error(ss.str());
    }
    return driver;
}

pdal::Stage* infer_and_create_reader(pdal::StageFactory& factory,
                                     const std::string& path) {
    pdal::Options options;
    options.add("filename", path);
    std::string driver = infer_reader_driver(factory, path);
    pdal::Stage* reader = factory.createStage(driver);
    reader->setOptions(options);
    return reader;
}

Matrix point_view_to_matrix(const pdal::PointViewPtr view) {
    Matrix matrix(view->size(), 3);
    for (pdal::point_count_t i = 0; i < view->size(); ++i) {
        matrix(i, 0) = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
        matrix(i, 1) = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
        matrix(i, 2) = view->getFieldAs<double>(pdal::Dimension::Id::Z, i);
    }
    return matrix;
}

double point_view_average_time(const pdal::PointViewPtr view) {
    double time = 0.0;
    for (pdal::point_count_t i = 0; i < view->size(); ++i) {
        time += view->getFieldAs<double>(pdal::Dimension::Id::GpsTime, i) /
                double(view->size());
    }
    return time;
}

CroppedFile::CroppedFile(const std::string& filename,
                         const pdal::BOX2D& bounds) {
    pdal::StageFactory factory(false);
    pdal::Stage* reader = infer_and_create_reader(factory, filename);
    pdal::Options options;
    options.add("bounds", bounds);
    pdal::CropFilter crop;
    crop.setInput(*reader);
    crop.setOptions(options);
    pdal::PointTable table;
    crop.prepare(table);
    pdal::PointViewSet viewset = crop.execute(table);
    assert(viewset.size() == 1);
    matrix = point_view_to_matrix(*viewset.begin());
    time = point_view_average_time(*viewset.begin());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr eigen_to_pcl(const Matrix& matrix) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = matrix.rows();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width);

    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud->points[i].x = float(matrix(i, 0));
        cloud->points[i].y = float(matrix(i, 1));
        cloud->points[i].z = float(matrix(i, 2));
    }

    return cloud;
}

Matrix pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    Matrix matrix(cloud.size(), 3);
    for (size_t i = 0; i < cloud.points.size(); ++i) {
        matrix(i, 0) = cloud.points[i].x;
        matrix(i, 1) = cloud.points[i].y;
        matrix(i, 2) = cloud.points[i].z;
    }
    return matrix;
}
