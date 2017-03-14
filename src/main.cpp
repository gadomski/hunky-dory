#include "Eigen/Dense"
#include "bounds.hpp"
#include "cpd.hpp"
#include "docopt.h"
#include "pdal/EigenUtils.hpp"
#include "pdal/PointTable.hpp"
#include "pdal/PointView.hpp"
#include "pdal/util/Bounds.hpp"
#include "read.hpp"
#include "write.hpp"

static const char USAGE[] =
    R"(Change detection for point clouds.

    I watch the ripples change their size
    But never leave the stream
    Of warm impermanence

        - David Bowie, "Changes"

Usage:
    hunky-dory <file1> <file2> <outfile> [--capacity=<n>] [--sigma2=<n>] [--limit=<n>] [--driver=<driver>] [--edge-length=<n>] [--radius=<n>] [--no-data=<n>] [--window-size=<n>] [--dump-bounds=<filename>]
    hunky-dory (-h | --help)

Options:
    -h --help                   Show this screen.
    --capacity=<n>              The (approximate) number of points in each chip [default: 10000].
    --driver=<name>             The GDAL driver to use for writing the output raster [default: GTiff].
    --edge-length=<n>           The length of each edge, in m, of the output raster [default: 5].
    --sigma2=<n>                The initial sigma2 (bandwidth) for cpd [default: 0.02].
    --limit=<n>                 The maximum number of chips to process (mostly for testing purposes). If zero, no limit. [default: 0].
    --radius=<n>                The influencing radius [default: 10.0].
    --no-data=<n>               The raster no data value [default: -9999.0].
    --window-size=<n>           The distance for fallback interpolation [default: 10.0].
    --dump-bounds=<filename>    If provided, write all chip bounds out to a file, one bounds per line.
)";

int main(int argc, char** argv) {
    std::map<std::string, docopt::value> args =
        docopt::docopt(USAGE, {argv + 1, argv + argc}, true, "");
    std::string filename1 = args.at("<file1>").asString();
    std::string filename2 = args.at("<file2>").asString();
    std::string outfile = args.at("<outfile>").asString();
    pdal::point_count_t capacity = args.at("--capacity").asLong();
    double sigma2 = std::stod(args.at("--sigma2").asString());
    size_t limit = args.at("--limit").asLong();
    std::string driver = args.at("--driver").asString();
    double edge_length = std::stod(args.at("--edge-length").asString());
    double radius = std::stod(args.at("--edge-length").asString());
    double no_data = std::stod(args.at("--no-data").asString());
    double window_size = std::stod(args.at("--window-size").asString());

    std::cout << "Calculating bounds" << std::endl;
    std::vector<pdal::Bounds> bounds = hunky_dory::bounds(filename1, capacity);
    std::cout << bounds.size() << " boxes computed" << std::endl;
    if (args.at("--dump-bounds")) {
        std::ofstream boundsfile(args.at("--dump-bounds").asString());
        for (auto bounds : bounds) {
            boundsfile << bounds << std::endl;
        }
    }

    std::cout << "Reading " << filename1 << std::endl;
    pdal::PointTable table1;
    pdal::PointViewSet chips1 = hunky_dory::read(filename1, table1, bounds);
    std::cout << "Reading " << filename2 << std::endl;
    pdal::PointTable table2;
    pdal::PointViewSet chips2 = hunky_dory::read(filename2, table2, bounds);
    assert(chips1.size() == chips2.size());

    std::vector<Eigen::MatrixXd> results;
    results.reserve(chips1.size());
    size_t i = 0;
    for (auto iter1 = chips1.begin(), iter2 = chips2.begin();
         iter1 != chips1.end() || iter2 != chips2.end(); ++iter1, ++iter2) {
        Eigen::MatrixXd data1 = pdal::eigen::pointViewToEigen(**iter1);
        if (data1.rows() == 0) {
            std::cout << "Data 1 empty, skipping." << std::endl;
            continue;
        }
        Eigen::MatrixXd data2 = pdal::eigen::pointViewToEigen(**iter2);
        if (data2.rows() == 0) {
            std::cout << "Data 2 empty, skipping." << std::endl;
            continue;
        }

        Eigen::MatrixXd result = hunky_dory::cpd_rigid(data2, data1, sigma2);
        std::cout << (result - data1).colwise().mean() << std::endl;
        results.push_back(result - data1);

        ++i;
        std::cout << "Chip " << i << " of " << chips1.size() << " processed"
                  << std::endl;
        if (limit != 0 && limit <= i) {
            std::cout << "Limit of " << limit << " reached, breaking"
                      << std::endl;
            break;
        }
    }

    hunky_dory::write(chips1, results, outfile, driver, edge_length, radius,
                      no_data, window_size);
    return 0;
}
