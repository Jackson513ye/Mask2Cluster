#pragma once

#include <vector>

#include "m2c/kdtree.h"
#include "m2c/types.h"

namespace m2c {

struct Cluster {
	std::vector<int> indices;  // Indices of points participating in the cluster.
	float diameter = 0.0f;     // Estimated bounding-box diameter for validation.
};

// Seeded DBSCAN growth using pure Euclidean neighborhoods.
// Core points expand their neighbors; boundary points join without further expansion.
// Diameter should be approximated via an axis-aligned bounding box.
Cluster growFromSeed_DBSCAN(int seed_idx,
														const CloudT& cloud,
														const KD& kd,
														float eps,
														int minPts_core,
														int maxPts,
														float maxDiameter);

}  // namespace m2c
