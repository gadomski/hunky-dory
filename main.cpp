#include <iostream>

#include <cpd/rigid.hpp>
#include <entwine/tree/builder.hpp>
#include <pdal/ChipperFilter.hpp>
#include <pdal/CropFilter.hpp>
#include <pdal/PointViewIter.hpp>
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
    hunky-dory cpd <source> <target> [--capacity=n] [--sigma2=n] [--no-entwine]
    hunky-dory (-h | --help)
    hunky-dory --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    --capacity=n    Approximate capacity of each tile/chip/segment. [default: 20000]
    --sigma2=n      Starting bandwidth for CPD alignment calculations. [default: 5.0]
    --no-entwine    Don't use entwine's index — use PDAL's chipper instead.
)";

pdal::Stage* infer_and_create_reader(pdal::StageFactory& factory,
                                     const std::string& path);
cpd::Matrix point_view_to_matrix(const pdal::PointViewPtr view);

int main(int argc, char** argv) {
    std::map<std::string, docopt::value> args = docopt::docopt(
        USAGE, {argv + 1, argv + argc}, true, hunky_dory::VERSION);

    std::string source_path = args.at("<source>").asString();
    std::string target_path = args.at("<target>").asString();
    long capacity = args.at("--capacity").asLong();
    double sigma2 = std::stod(args.at("--sigma2").asString());

    if (args.at("--no-entwine").asBool()) {
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

        pdal::PointTable source_table;
        chipper.prepare(source_table);

        std::cout << "Chipping..." << std::flush;
        pdal::PointViewSet viewset = chipper.execute(source_table);
        std::cout << "done\n";

        for (auto it = viewset.begin(); it != viewset.end(); ++it) {
            pdal::PointViewPtr source_view = *it;
            cpd::Matrix source = point_view_to_matrix(source_view);
            pdal::BOX2D bounds;
            source_view->calculateBounds(bounds);
            std::cout << "Iteration with source bounds: " << bounds << "\n";

            pdal::Options crop_options;
            crop_options.add("bounds", bounds);
            pdal::CropFilter crop;
            crop.setInput(*target_reader);
            crop.setOptions(crop_options);

            pdal::PointTable target_table;
            crop.prepare(target_table);

            std::cout << "Cropping target data..." << std::flush;
            pdal::PointViewSet target_viewset = crop.execute(target_table);
            std::cout << "done\n";

            assert(target_viewset.size() == 1);
            pdal::PointViewPtr target_view = *target_viewset.begin();
            cpd::Matrix target = point_view_to_matrix(target_view);

            cpd::Rigid registration = cpd::Rigid();
            registration.use_fgt(false);
            cpd::RigidResult result = registration.compute(source, target, sigma2);
            std::cout << "CPD rigid translation: \n"
                      << result.translation << "\n";
            break;
        }
    } else {
        std::cout << "Using entwine indices\n";
        entwine::Builder builder(source_path);
        entwine::Schema xyz({entwine::DimInfo("X", "floating", 8),
                             entwine::DimInfo("Y", "floating", 8),
                             entwine::DimInfo("Z", "floating", 8)});
        auto handler([&](pdal::PointView& view, entwine::BBox bbox) {
            std::cout << "Number of points: " << view.size() << "\n";
            return true;
        });

        builder.traverse(1, 100 * 100, handler, &xyz);
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

cpd::Matrix point_view_to_matrix(const pdal::PointViewPtr view) {
    cpd::Matrix matrix(view->size(), 3);
    for (pdal::point_count_t i = 0; i < view->size(); ++i) {
        matrix(i, 0) = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
        matrix(i, 1) = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
        matrix(i, 2) = view->getFieldAs<double>(pdal::Dimension::Id::Z, i);
    }
    return matrix;
}
