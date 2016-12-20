#include "bounds.hpp"
#include "create_reader.hpp"
#include "pdal/filters/ChipperFilter.hpp"

namespace hunky_dory {
std::vector<pdal::Bounds> bounds(const std::string& filename,
                                 pdal::point_count_t capacity) {
    pdal::StageFactory factory;
    pdal::Stage* reader = create_reader(factory, filename);
    pdal::ChipperFilter chipper;
    chipper.setInput(*reader);
    pdal::Options options;
    options.add("capacity", capacity);
    chipper.setOptions(options);
    pdal::PointTable table;
    chipper.prepare(table);
    pdal::PointViewSet pointViewSet = chipper.execute(table);

    std::vector<pdal::Bounds> bounds;
    bounds.reserve(pointViewSet.size());
    for (auto pointView : pointViewSet) {
        pdal::BOX2D box;
        pointView->calculateBounds(box);
        bounds.push_back(pdal::Bounds(box));
    }
    return bounds;
}
}
