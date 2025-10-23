#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "m2c/io_las.h"
#include "m2c/kdtree.h"

namespace {

void printUsage(const char* prog) {
  std::cout << "Usage: " << prog << " --in <point_cloud.{las|ply|pcd}> --radius <meters>" << std::endl;
}

struct Args {
  std::string cloud_path;
  float radius = 0.5f;
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
    } else if (current == "--radius") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --radius");
      }
      args.radius = std::stof(argv[++i]);
    } else {
      throw std::runtime_error("Unknown argument: " + current);
    }
  }
  if (args.cloud_path.empty()) {
    throw std::runtime_error("--in must be provided");
  }
  if (args.radius <= 0.0f) {
    throw std::runtime_error("--radius must be positive");
  }
  return args;
}

int randomIndex(std::size_t upper) {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<std::size_t> dist(0, upper - 1);
  return static_cast<int>(dist(gen));
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
    m2c::CloudT::Ptr cloud = m2c::loadAnyPointCloud(args.cloud_path);
    if (cloud->empty()) {
      std::cerr << "Cloud is empty, nothing to query" << std::endl;
      return 1;
    }

    m2c::KD kd(*cloud);
    const int seed_idx = randomIndex(cloud->size());

    std::vector<int> neighbors;
    neighbors.reserve(128);
    kd.radius(seed_idx, args.radius, neighbors);

    std::cout << "Cloud size       : " << cloud->size() << "\n";
    std::cout << "Query index      : " << seed_idx << "\n";
    std::cout << "Radius (meters)  : " << args.radius << "\n";
    std::cout << "Neighbor count   : " << neighbors.size() << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "KD probe failed: " << e.what() << std::endl;
    return 1;
  }
}
