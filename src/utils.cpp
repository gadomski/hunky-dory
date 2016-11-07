#include <entwine/reader/cache.hpp>
#include <entwine/reader/query.hpp>
#include <entwine/reader/reader.hpp>
#include <entwine/types/vector-point-table.hpp>
#include <pdal/CropFilter.hpp>

#include "utils.hpp"

namespace hunky_dory {
std::string infer_reader_driver(const pdal::StageFactory& factory,
                                const std::string& path) {
    return factory.inferReaderDriver(path);
}

pdal::Stage* infer_and_create_reader(pdal::StageFactory& factory,
                                     const std::string& path) {
    pdal::Options options;
    options.add("filename", path);
    std::string driver = infer_reader_driver(factory, path);
    if (driver.empty()) {
        return nullptr;
    }
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
    if (!reader) {
        // try entwine
        assert(false);
    } else {
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

entwine::Bounds box2d_to_bbox(const pdal::BOX2D& box2d) {
    entwine::Point min(box2d.minx, box2d.miny);
    entwine::Point max(box2d.maxx, box2d.maxy);
    return entwine::Bounds(min, max);
}
}
