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
    hunky-dory cpd chip <source> <target> <outfile> [--capacity=n] [--sigma2=n] [--no-entwine]
    hunky-dory cpd bounds <source> <target> <bounds> [--sigma2=n]
    hunky-dory icp bounds <source> <target> <bounds>
    hunky-dory (-h | --help)
    hunky-dory --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    --capacity=n    Approximate capacity of each tile/chip/segment. [default: 20000]
    --sigma2=n      Starting bandwidth for CPD alignment calculations. [default: 5.0]
    --no-entwine    Don't use entwine's index — use PDAL's chipper instead.
)";

typedef fgt::Matrix Matrix;
typedef std::map<std::string, docopt::value> DocoptMap;

pdal::Stage* infer_and_create_reader(pdal::StageFactory&, const std::string&);
Matrix point_view_to_matrix(const pdal::PointViewPtr);
struct CroppedFile {
    CroppedFile(const std::string&, const pdal::BOX2D&);
    Matrix matrix;
    double time;
};
int cpd_chip(const DocoptMap&);
int cpd_bounds(const DocoptMap&);
int icp_bounds(const DocoptMap&);

int main(int argc, char** argv) {
    DocoptMap args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true,
                                    hunky_dory::VERSION);

    if (args.at("cpd").asBool()) {
        if (args.at("chip").asBool()) {
            return cpd_chip(args);
        } else if (args.at("bounds").asBool()) {
            return cpd_bounds(args);
        } else {
            std::cerr << "Unsupported cpd method." << std::endl;
            return 1;
        }
    } else if (args.at("icp").asBool()) {
        if (args.at("bounds").asBool()) {
            return icp_bounds(args);
        } else {
            std::cerr << "Unsupported icp method." << std::endl;
            return 1;
        }
    } else {
        std::cerr << "Unsupported registration method." << std::endl;
        return 1;
    }
}

int cpd_chip(const DocoptMap& args) {
    std::string source_path = args.at("<source>").asString();
    std::string target_path = args.at("<target>").asString();
    std::ofstream outfile(args.at("<outfile>").asString());
    outfile.precision(2);
    outfile << std::fixed;
    double sigma2 = std::stod(args.at("--sigma2").asString());
    long capacity = args.at("--capacity").asLong();

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
        std::cout << "done, with " << viewset.size() << " chips\n";

        for (auto source_view : viewset) {
            Matrix source = point_view_to_matrix(source_view);
            pdal::BOX2D bounds;
            source_view->calculateBounds(bounds);

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
            Matrix target = point_view_to_matrix(target_view);

            cpd::Options options;
            options.set_sigma2(sigma2);
            cpd::RigidResult<Matrix> result =
                cpd::rigid(source, target, options);
            fgt::Vector translation =
                (target - result.moving).colwise().mean().transpose();
            {
                std::cout << "Runtime: " << result.runtime
                          << "s\nAverage motion:\n"
                          << translation << "\n";
                outfile << bounds.minx << " " << bounds.maxx << " "
                        << bounds.miny << " " << bounds.maxy << " "
                        << translation(0) << " " << translation(1) << " "
                        << translation(2) << "\n";
                outfile << std::flush;
            }
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

        builder.traverse(1, 100, handler, &xyz);
    }
    outfile.close();
    return 0;
}

int cpd_bounds(const DocoptMap& args) {
    std::stringstream bounds_ss(args.at("<bounds>").asString());
    pdal::BOX2D bounds;
    bounds_ss >> bounds;
    if (bounds.empty()) {
        std::cerr << "Invalid bounds" << std::endl;
        return 1;
    }

    std::cerr << "Cropping source file..." << std::flush;
    CroppedFile source_file(args.at("<source>").asString(), bounds);
    Matrix source = source_file.matrix;
    std::cerr << "done with " << source.rows()
              << " points.\nCropping target file..." << std::flush;
    CroppedFile target_file(args.at("<target>").asString(), bounds);
    Matrix target = target_file.matrix;
    std::cerr << "done with " << target.rows() << " points.\n";
    double sigma2 = std::stod(args.at("--sigma2").asString());

    cpd::Options options(std::cerr);
    options.set_sigma2(sigma2);
    cpd::RigidResult<Matrix> result = cpd::rigid(source, target, options);

    fgt::Vector translation = (target - result.moving).colwise().mean();
    std::cerr << "Runtime: " << result.runtime
              << "s\nMotion:" << translation.transpose() << "\n";

    std::cout << std::fixed;
    std::cout << "{\"runtime\": " << result.runtime
              << ", \"num_source\": " << source.rows()
              << ", \"source_time\": " << source_file.time
              << ", \"num_target\": " << target.rows()
              << ", \"target_time\": " << target_file.time
              << ", \"iterations\": " << result.iterations
              << ", \"minx\": " << bounds.minx << ", \"maxx\": " << bounds.maxx
              << ", \"miny\": " << bounds.miny << ", \"maxy\": " << bounds.maxy
              << ", \"dx\": " << translation(0)
              << ", \"dy\": " << translation(1)
              << ", \"dz\": " << translation(2) << "}\n";

    return 0;
}

int icp_bounds(const DocoptMap& args) { return 0; }

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
