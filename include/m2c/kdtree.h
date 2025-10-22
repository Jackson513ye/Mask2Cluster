#pragma once

#include <vector>

#include "m2c/types.h"

namespace m2c {

// Thin wrapper around pcl::search::KdTree<PointT> to support radius queries.
// Callers should preallocate the output index buffer to minimize reallocations.
struct KD {
	explicit KD(const CloudT& cloud);

	void radius(int idx, float r, std::vector<int>& out) const;
};

}  // namespace m2c
