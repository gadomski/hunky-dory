#include "read.hpp"
#include "create_reader.hpp"
#include "pdal/StageFactory.hpp"

namespace hunky_dory {

pdal::PointViewSet read(const std::string& filename, pdal::PointTable& table,
                        const std::vector<pdal::Bounds>& bounds) {
    pdal::StageFactory factory;
    pdal::Stage* reader = create_reader(factory, filename);
    pdal::Stage* crop = factory.createStage("filters.crop");
    crop->setInput(*reader);
    pdal::Options options;
    for (auto bounds : bounds) {
        options.add("bounds", bounds);
    }
    crop->setOptions(options);
    crop->prepare(table);
    return crop->execute(table);
}
}
