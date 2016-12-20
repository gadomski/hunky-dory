#include "write.hpp"
#include "pdal/GDALUtils.hpp"
#include "pdal/io/GDALGrid.hpp"

namespace hunky_dory {

const size_t NUM_BANDS = 3;

void write_band(pdal::gdal::Raster& raster, pdal::GDALGrid& grid, int number,
                const std::string& name) {
    uint8_t* buf;
    buf = grid.data("mean");
    assert(buf);
    raster.writeBand(buf, number, name);
}

void write(const pdal::PointViewSet& pointViewSet,
           const std::vector<Eigen::MatrixXd>& results,
           const std::string& outfile, const std::string& driver,
           double edge_length, double radius, double no_data,
           double window_size) {
    GDALAllRegister();
    pdal::BOX2D bounds;
    pdal::PointView::calculateBounds(pointViewSet, bounds);
    size_t width = (bounds.maxx - bounds.minx) / edge_length + 1;
    size_t height = (bounds.maxy - bounds.miny) / edge_length + 1;

    pdal::GDALGrid xgrid(width, height, edge_length, radius, no_data,
                         pdal::GDALGrid::statMean, window_size);
    pdal::GDALGrid ygrid(width, height, edge_length, radius, no_data,
                         pdal::GDALGrid::statMean, window_size);
    pdal::GDALGrid zgrid(width, height, edge_length, radius, no_data,
                         pdal::GDALGrid::statMean, window_size);

    assert(pointViewSet.size() == results.size());
    auto pointViewIter = pointViewSet.begin();
    auto resultsIter = results.begin();
    for (; pointViewIter != pointViewSet.end() && resultsIter != results.end();
         ++pointViewIter, ++resultsIter) {
        auto pointView = **pointViewIter;
        auto result = *resultsIter;
        assert(pointView.size() == result.rows());
        pdal::point_count_t npoints = pointView.size();
        for (pdal::point_count_t i = 0; i < npoints; ++i) {
            double x = pointView.getFieldAs<double>(pdal::Dimension::Id::X, i) -
                       bounds.minx;
            double y = pointView.getFieldAs<double>(pdal::Dimension::Id::Y, i) -
                       bounds.miny;
            xgrid.addPoint(x, y, result(i, 0));
            ygrid.addPoint(x, y, result(i, 1));
            zgrid.addPoint(x, y, result(i, 2));
        }
    }
    xgrid.finalize();
    ygrid.finalize();
    zgrid.finalize();

    std::array<double, 6> pixel_to_pos = {
        bounds.minx, edge_length, 0, bounds.miny + (edge_length * height),
        0,           -edge_length};
    pdal::gdal::Raster raster(outfile, driver,
                              (*pointViewSet.begin())->spatialReference(),
                              pixel_to_pos);
    pdal::gdal::GDALError err = raster.open(
        width, height, NUM_BANDS, pdal::Dimension::Type::Double, no_data);
    if (err != pdal::gdal::GDALError::None) {
        throw std::runtime_error(raster.errorMsg());
    }
    write_band(raster, xgrid, 1, "xvelocity");
    write_band(raster, ygrid, 2, "yvelocity");
    write_band(raster, zgrid, 3, "zvelocity");
}
}
