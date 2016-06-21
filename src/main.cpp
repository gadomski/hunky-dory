#include <iostream>

#include <cpd/rigid.hpp>
#include <entwine/tree/tiler.hpp>
#include <entwine/types/schema.hpp>
#include <pdal/ChipperFilter.hpp>
#include <pdal/PointViewIter.hpp>

#include "cpd.hpp"
#include "hunky_dory.hpp"
#include "icp.hpp"
#include "utils.hpp"

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

int chip(const hunky_dory::DocoptMap&);
int bounds(const hunky_dory::DocoptMap&);

int main(int argc, char** argv) {
    hunky_dory::DocoptMap args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true,
                                    hunky_dory::VERSION);

    if (args.at("chip").asBool()) {
        return chip(args);
    } else if (args.at("bounds").asBool()) {
        return bounds(args);
    } else {
        std::cerr << "Unsupported action." << std::endl;
        return 1;
    }
}

// TODO not actually generalized for multiple methods
int chip(const hunky_dory::DocoptMap& args) {
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
            hunky_dory::infer_and_create_reader(factory, source_path);
        pdal::Stage* target_reader =
            hunky_dory::infer_and_create_reader(factory, target_path);

        pdal::Options chipper_options;
        chipper_options.add("capacity", capacity);
        pdal::ChipperFilter chipper;
        chipper.setInput(*source_reader);
        chipper.setOptions(chipper_options);

        pdal::PointTable source_table;
        chipper.prepare(source_table);

        std::cout << "Chipping..." << std::flush;
        pdal::PointViewSet viewset = chipper.execute(source_table);
        std::cout << "done, with " << viewset.size() << " chips\n";

        for (auto source_view : viewset) {
            hunky_dory::Matrix source = hunky_dory::point_view_to_matrix(source_view);
            pdal::BOX2D bounds;
            source_view->calculateBounds(bounds);

            std::cout << "Cropping target data..." << std::flush;
            hunky_dory::CroppedFile target_file(target_path, bounds);
            std::cout << "done\n";
            hunky_dory::Matrix target = target_file.matrix;

            cpd::Options options;
            options.set_sigma2(sigma2);
            cpd::RigidResult<hunky_dory::Matrix> result =
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
        entwine::arbiter::Arbiter a;
        const entwine::arbiter::Endpoint source(a.getEndpoint(source_path));
        entwine::Schema xyz({entwine::DimInfo("X", "floating", 8),
                             entwine::DimInfo("Y", "floating", 8),
                             entwine::DimInfo("Z", "floating", 8)});
        entwine::Tiler t(source, 6, 1000, &xyz);
        auto handler([&](pdal::PointView& view, entwine::BBox bbox) {
            std::cout << "Number of points: " << view.size() << "\n";
            return true;
        });

        t.go(handler);
    }
    outfile.close();
    return 0;
}

int bounds(const hunky_dory::DocoptMap& args) {
    std::stringstream bounds_ss(args.at("<bounds>").asString());
    pdal::BOX2D bounds;
    bounds_ss >> bounds;
    if (bounds.empty()) {
        std::cerr << "Invalid bounds" << std::endl;
        return 1;
    }

    std::cerr << "Cropping source file..." << std::flush;
    hunky_dory::CroppedFile source_file(args.at("<source>").asString(), bounds);
    hunky_dory::Matrix source = source_file.matrix;
    std::cerr << "done with " << source.rows()
              << " points.\nCropping target file..." << std::flush;
    hunky_dory::CroppedFile target_file(args.at("<target>").asString(), bounds);
    hunky_dory::Matrix target = target_file.matrix;
    std::cerr << "done with " << target.rows() << " points.\n";

    hunky_dory::Result result;
    if (args.at("cpd").asBool()) {
        result = hunky_dory::cpd(source, target, args);
    } else if (args.at("icp").asBool()) {
        result = hunky_dory::icp(source, target, args);
    } else {
        std::cerr << "Unsupported method." << std::endl;
        return 1;
    }
    std::cout << std::fixed;
    std::cout << "{\"runtime\": " << result.runtime
              << ", \"num_source\": " << source.rows()
              << ", \"source_time\": " << source_file.time
              << ", \"num_target\": " << target.rows()
              << ", \"target_time\": " << target_file.time
              << ", \"iterations\": " << result.iterations
              << ", \"minx\": " << bounds.minx << ", \"maxx\": " << bounds.maxx
              << ", \"miny\": " << bounds.miny << ", \"maxy\": " << bounds.maxy
              << ", \"dx\": " << result.dx << ", \"dy\": " << result.dy
              << ", \"dz\": " << result.dz << "}\n";

    return 0;
}
