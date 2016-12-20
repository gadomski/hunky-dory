#include "create_reader.hpp"

namespace hunky_dory {
pdal::Stage* create_reader(pdal::StageFactory& factory,
                           const std::string& filename) {
    std::string driver = factory.inferReaderDriver(filename);
    pdal::Stage* reader = factory.createStage(driver);
    pdal::Options options;
    options.add("filename", filename);
    reader->setOptions(options);
    return reader;
}
}
