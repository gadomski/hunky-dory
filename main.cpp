#include <iostream>

#include "docopt.h"
#include "hunky_dory.hpp"

static const char USAGE[] =
    R"(Change detection for large point clouds.

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
    std::cout << "I watch the ripples change their size\nBut never leave the "
                 "stream\nOf warm impermanence\n";
    return 0;
}
