#pragma once

#include "m2c/dbscan_seeded.h"
#include "m2c/types.h"
#include "m2c/validator.h"

namespace m2c {

struct Result {
	bool found = false;  // True when a qualifying cluster is produced.
	int trials = 0;      // Number of seed attempts made.
	Cluster cluster;     // Captured cluster (valid when found == true).
};

// Orchestrate seeded cluster selection around reference point C.
// Points are processed in ascending distance to C, with soft bans preventing reuse of failed clusters.
// Iteration halts once a cluster passes validation or the max_trials limit is reached.
Result selectCluster(const CloudT& cloud, const Pose& pose, const Params& params);

}  // namespace m2c
