#include <iostream>

#include <pdal/ChipperFilter.hpp>
#include <pdal/StageFactory.hpp>

#include "docopt.h"
#include "hunky_dory.hpp"

static const char USAGE[] =
    R"(Change detection for large point clouds.

    I watch the ripples change their size
    But never leave the stream
    Of warm impermanence

        - David Bowie, "Changes"

Usage:
    hunky-dory cpd <source> <target> [--capacity=n] [--no-entwine]
    hunky-dory (-h | --help)
    hunky-dory --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    --capacity=n    Approximate capacity of each tile/chip/segment. [default: 20000]
    --no-entwine    Don't use entwine's index — use PDAL's chipper instead.
)";

pdal::Stage* infer_and_create_reader(pdal::StageFactory& factory,
                                     const std::string& path);

int main(int argc, char** argv) {
    std::map<std::string, docopt::value> args = docopt::docopt(
        USAGE, {argv + 1, argv + argc}, true, hunky_dory::VERSION);

    std::string source_path = args.at("<source>").asString();
    std::string target_path = args.at("<target>").asString();
    long capacity = args.at("--capacity").asLong();

    if (args["--no-entwine"]) {
        std::cout << "Using PDAL's chipper\n";

        pdal::StageFactory factory(false);

        pdal::Stage* source_reader =
            infer_and_create_reader(factory, source_path);
        pdal::Stage* target_reader =
            infer_and_create_reader(factory, target_path);

        pdal::Options chipper_options;
        chipper_options.add("capacity", capacity, "");

        pdal::ChipperFilter chipper;
        chipper.setInput(*source_reader);
        chipper.setOptions(chipper_options);

        pdal::PointTable table;
        chipper.prepare(table);

        pdal::PointViewSet viewset = chipper.execute(table);

    } else {
        std::cout << "Using entwine indices\n";
        std::cout << "ERROR: not supported yet...\n";
        return 1;
    }
    return 0;
}

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
