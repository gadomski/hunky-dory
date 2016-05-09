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
    hunky-dory cpd <source> <target>
    hunky-dory (-h | --help)
    hunky-dory --version

Options:
    -h --help       Show this screen.
    --version       Show version.
)";

int main(int argc, char** argv) {
    auto args = docopt::docopt(USAGE, {argv + 1, argv + argc}, true,
                               hunky_dory::VERSION);
    std::string source_path = args["source"].asString();
    std::string target_path = args["target"].asString();
    return 0;
}
