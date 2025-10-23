#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#include "m2c/io_las.h"
#include "m2c/io_pose.h"

namespace {

void printUsage(const char* prog) {
  std::cout << "Usage: " << prog
            << " --in <point_cloud.{las|ply|pcd}> --pose <pose.json>" << std::endl;
}

struct Args {
  std::string cloud_path;
  std::string pose_path;
};

Args parseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string current(argv[i]);
    if (current == "--help" || current == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (current == "--in") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --in");
      }
      args.cloud_path = argv[++i];
    } else if (current == "--pose") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --pose");
      }
      args.pose_path = argv[++i];
    } else {
      throw std::runtime_error("Unknown argument: " + current);
    }
  }

  if (args.cloud_path.empty() || args.pose_path.empty()) {
    throw std::runtime_error("Both --in and --pose must be provided");
  }

  return args;
}

}  // namespace

int main(int argc, char** argv) {
  Args args;
  try {
    args = parseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Argument error: " << e.what() << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  try {
    const m2c::Pose pose = m2c::loadPoseJSON(args.pose_path);
    m2c::CloudT::Ptr cloud = m2c::loadAnyPointCloud(args.cloud_path);

    std::cout << "Loaded point cloud: " << args.cloud_path << "\n";
    std::cout << "Point count    : " << cloud->size() << "\n";
    std::cout << "Reference C    : [" << pose.C.x() << ", " << pose.C.y() << ", " << pose.C.z()
              << "]\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Loader probe failed: " << e.what() << std::endl;
    return 1;
  }
}
