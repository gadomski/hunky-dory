#include "bounds.hpp"
#include "chip.hpp"
#include "hunky_dory.hpp"
#include "utils.hpp"

static const char USAGE[] =
    R"(Change detection for large point clouds.

    I watch the ripples change their size
    But never leave the stream
    Of warm impermanence

        - David Bowie, "Changes"

Usage:
    hunky-dory cpd chip <source> <target> <outfile> [--capacity=n] [--sigma2=n] [--no-entwine]
    hunky-dory cpd bounds <source> <target> <bounds> [--sigma2=n] [--outliers=n]
    hunky-dory icp chip <source> <target> <outfile> [--capacity=n] [--no-entwine]
    hunky-dory icp bounds <source> <target> <bounds>
    hunky-dory (-h | --help)
    hunky-dory --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    --capacity=n    Approximate capacity of each tile/chip/segment. [default: 20000]
    --sigma2=n      Starting bandwidth for CPD alignment calculations. [default: 5.0]
    --outliers=n     Outlier weight for CPD alignment calculations. [default: 0.1]
    --no-entwine    Don't use entwine's index — use PDAL's chipper instead.
)";

int main(int argc, char** argv) {
    hunky_dory::DocoptMap args = docopt::docopt(USAGE, {argv + 1, argv + argc},
                                                true, hunky_dory::VERSION);

    if (args.at("chip").asBool()) {
        return hunky_dory::chip(args);
    } else if (args.at("bounds").asBool()) {
        return hunky_dory::bounds(args);
    } else {
        std::cerr << "Unsupported action." << std::endl;
        return 1;
    }
}
