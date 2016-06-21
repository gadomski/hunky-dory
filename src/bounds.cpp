#include "bounds.hpp"
#include "cpd.hpp"
#include "icp.hpp"

namespace hunky_dory {
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
}
