#include <iostream>

#include "docopt.h"
#include "hunky_dory.hpp"

static const char USAGE[] =
    R"(Change detection for large point clouds.

    I watch the ripples change their size
    But never leave the stream
    Of warm impermanence

        - David Bowie, "Changes"

Usage:
    hunky-dory cpd <source> <target> [--no-entwine]
    hunky-dory (-h | --help)
    hunky-dory --version

Options:
    -h --help       Show this screen.
    --version       Show version.
    --no-entwine    Don't use entwine's index — use PDAL's chipper instead.
)";

int main(int argc, char** argv) {
    std::map<std::string, docopt::value> args = docopt::docopt(
        USAGE, {argv + 1, argv + argc}, true, hunky_dory::VERSION);
    std::string source_path = args.at("<source>").asString();
    std::string target_path = args.at("<target>").asString();

    if (args["--no-entwine"]) {
        std::cout << "Using PDAL's chipper\n";
    } else {
        std::cout << "Using entwine indices\n";
        std::cout << "ERROR: not supported yet...\n";
        return 1;
    }
    return 0;
}
