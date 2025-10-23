#include <cstdint>
#include <cstdlib>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

#include "m2c/io_las.h"
#include "m2c/io_pose.h"
#include "m2c/pipeline.h"

namespace {

using m2c::Params;

struct CLIOptions {
  std::string cloud_path;
  std::string pose_path;
  std::string output_path;
  std::string config_path;

  std::optional<float> eps;
  std::optional<int> minPts_core;
  std::optional<int> minPts_total;
  std::optional<float> maxDiameter;
  std::optional<int> maxPts;
  std::optional<int> max_trials;
  std::optional<float> voxel;
  std::optional<float> n;   // optional override for fraction multiplier
  std::optional<int> m;     // optional override for top-M voting
};

void printUsage(const char* prog) {
  std::cout << "Usage: " << prog
            << " --in <point_cloud.{las|ply|pcd}> --pose <pose.json> --out <cluster.ply>"
            << " [--config <path.yaml>] [--eps <float>] [--minPtsCore <int>]"
            << " [--minPtsTotal <int>] [--maxDiameter <float>] [--maxPts <int>]"
            << " [--maxTrials <int>] [--voxel <float>] [--n <float>] [--m <int>]" << std::endl;
}

float parseFloat(const std::string& value, const std::string& name) {
  try {
    return std::stof(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid float for " + name + ": " + value);
  }
}

int parseInt(const std::string& value, const std::string& name) {
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid integer for " + name + ": " + value);
  }
}

CLIOptions parseArgs(int argc, char** argv) {
  CLIOptions opts;

  for (int i = 1; i < argc; ++i) {
    const std::string current(argv[i]);

    if (current == "--help" || current == "-h") {
      printUsage(argv[0]);
      std::exit(0);
    } else if (current == "--in") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --in");
      }
      opts.cloud_path = argv[++i];
    } else if (current == "--pose") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --pose");
      }
      opts.pose_path = argv[++i];
    } else if (current == "--out") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --out");
      }
      opts.output_path = argv[++i];
    } else if (current == "--config") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --config");
      }
      opts.config_path = argv[++i];
    } else if (current == "--eps") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --eps");
      }
      opts.eps = parseFloat(argv[++i], "--eps");
    } else if (current == "--minPtsCore") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --minPtsCore");
      }
      opts.minPts_core = parseInt(argv[++i], "--minPtsCore");
    } else if (current == "--minPtsTotal") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --minPtsTotal");
      }
      opts.minPts_total = parseInt(argv[++i], "--minPtsTotal");
    } else if (current == "--maxDiameter") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --maxDiameter");
      }
      opts.maxDiameter = parseFloat(argv[++i], "--maxDiameter");
    } else if (current == "--maxPts") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --maxPts");
      }
      opts.maxPts = parseInt(argv[++i], "--maxPts");
    } else if (current == "--maxTrials") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --maxTrials");
      }
      opts.max_trials = parseInt(argv[++i], "--maxTrials");
    } else if (current == "--voxel") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --voxel");
      }
      opts.voxel = parseFloat(argv[++i], "--voxel");
    } else if (current == "--n") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --n");
      }
      opts.n = parseFloat(argv[++i], "--n");
    } else if (current == "--m") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --m");
      }
      opts.m = parseInt(argv[++i], "--m");
    } else {
      throw std::runtime_error("Unknown argument: " + current);
    }
  }

  if (opts.cloud_path.empty() || opts.pose_path.empty() || opts.output_path.empty()) {
    throw std::runtime_error("--in, --pose, and --out are required");
  }

  return opts;
}

Params defaultParams() {
  Params params{};
  params.eps = 0.35f;
  params.minPts_core = 8;
  params.minPts_total = 60; // This line remains unchanged
  params.maxDiameter = 1.5f;
  params.maxPts = 500000;
  params.max_trials = 100;
  params.voxel = 0.05f;
  params.n = 0.25f; // New parameter
  params.m = 100;   // New parameter
  return params;
}

std::string trimCopy(const std::string& s) {
  const auto start = s.find_first_not_of(" \t\r\n");
  if (start == std::string::npos) {
    return std::string();
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(start, end - start + 1);
}

float parseScalar(const std::string& key, const std::string& value) {
  try {
    return std::stof(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid numeric value for '" + key + "': " + value);
  }
}

void applyYamlConfig(const std::string& path, Params& params) {
  if (path.empty()) {
    return;
  }

  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Failed to open config file: " + path);
  }

  std::string line;
  bool inCluster = false;

  while (std::getline(input, line)) {
    const std::string trimmed = trimCopy(line);
    if (trimmed.empty() || trimmed[0] == '#') {
      continue;
    }

    const bool topLevel = !line.empty() && !std::isspace(static_cast<unsigned char>(line[0]));
    if (topLevel && trimmed.back() == ':') {
      inCluster = trimmed == "cluster:";
      continue;
    }

    if (!inCluster) {
      continue;
    }

    const auto colonPos = trimmed.find(':');
    if (colonPos == std::string::npos) {
      continue;
    }

    std::string key = trimCopy(trimmed.substr(0, colonPos));
    std::string value = trimCopy(trimmed.substr(colonPos + 1));

    const auto commentPos = value.find('#');
    if (commentPos != std::string::npos) {
      value = trimCopy(value.substr(0, commentPos));
    }

    if (value.empty()) {
      continue;
    }

    if (key == "eps") {
      params.eps = parseScalar(key, value);
    } else if (key == "minPts_core") {
      params.minPts_core = static_cast<int>(parseScalar(key, value));
    } else if (key == "minPts_total") {
      params.minPts_total = static_cast<int>(parseScalar(key, value));
    } else if (key == "maxDiameter") {
      params.maxDiameter = parseScalar(key, value);
    } else if (key == "maxPts") {
      params.maxPts = static_cast<int>(parseScalar(key, value));
    } else if (key == "max_trials") {
      params.max_trials = static_cast<int>(parseScalar(key, value));
    } else if (key == "voxel") {
      params.voxel = parseScalar(key, value);
    }
  }
}

void applyOverrides(const CLIOptions& opts, Params& params) {
  if (opts.eps) {
    params.eps = *opts.eps;
  }
  if (opts.minPts_core) {
    params.minPts_core = *opts.minPts_core;
  }
  if (opts.minPts_total) {
    params.minPts_total = *opts.minPts_total;
  }
  if (opts.maxDiameter) {
    params.maxDiameter = *opts.maxDiameter;
  }
  if (opts.maxPts) {
    params.maxPts = *opts.maxPts;
  }
  if (opts.max_trials) {
    params.max_trials = *opts.max_trials;
  }
  if (opts.voxel) {
    params.voxel = *opts.voxel;
  }
}

void ensureOutputDirectory(const std::string& path) {
  const std::filesystem::path outPath(path);
  const auto parent = outPath.parent_path();
  if (!parent.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      throw std::runtime_error("Failed to create output directory: " + parent.string());
    }
  }
}

}  // namespace

int main(int argc, char** argv) {
  CLIOptions opts;
  try {
    opts = parseArgs(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "Argument error: " << e.what() << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  Params params = defaultParams();

  try {
    // New overrides for n and m
    if (opts.n) {
      params.n = *opts.n;
    }
    if (opts.m) {
      params.m = *opts.m;
    }
    applyYamlConfig(opts.config_path, params);
    applyOverrides(opts, params);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  if (params.minPts_core <= 0 || params.minPts_total <= 0 || params.maxPts <= 0 || params.max_trials <= 0) {
    std::cerr << "Configuration error: minPtsCore, minPtsTotal, maxPts, and maxTrials must be positive." << std::endl;
    return 1;
  }

  try {
    m2c::CloudT::Ptr cloud = m2c::loadAnyPointCloud(opts.cloud_path);
    const m2c::Pose pose = m2c::loadPoseJSON(opts.pose_path);

    m2c::CloudT::Ptr working = cloud;
    m2c::CloudT::Ptr filtered(new m2c::CloudT);

    if (params.voxel > 0.0f) {
      pcl::VoxelGrid<m2c::PointT> voxel;
      voxel.setInputCloud(cloud);
      voxel.setLeafSize(params.voxel, params.voxel, params.voxel);
      voxel.filter(*filtered);

      if (!filtered->empty()) {
        working = filtered;
      } else {
        std::cerr << "Warning: voxel downsampling produced an empty cloud; falling back to raw input." << std::endl;
      }
    }

    const m2c::Result selection = m2c::selectCluster(*working, pose, params);
    if (!selection.found) {
      std::cerr << "No qualifying cluster found after " << selection.trials << " trials." << std::endl;
      return 2;
    }

    if (selection.cluster.indices.empty()) {
      std::cerr << "Internal error: cluster reported as found but has no points." << std::endl;
      return 3;
    }

    m2c::CloudT output;
    output.reserve(selection.cluster.indices.size());
    for (int idx : selection.cluster.indices) {
      if (idx < 0 || static_cast<std::size_t>(idx) >= working->size()) {
        continue;
      }
      output.push_back((*working)[idx]);
    }

    if (output.empty()) {
      std::cerr << "Cluster extraction yielded no valid points." << std::endl;
      return 3;
    }

    output.width = static_cast<std::uint32_t>(output.size());
    output.height = 1;
    output.is_dense = false;

    ensureOutputDirectory(opts.output_path);
    if (pcl::io::savePLYFileBinary(opts.output_path, output) < 0) {
      std::cerr << "Failed to write output PLY: " << opts.output_path << std::endl;
      return 4;
    }

    std::cout << "Cluster saved to " << opts.output_path << " (" << output.size() << " points)" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Execution failed: " << e.what() << std::endl;
    return 5;
  }
}
