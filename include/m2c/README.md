# `include/m2c`

This directory will host the public C++ headers for the mask2cluster pipeline. Only interface declarations will live here; implementation files will remain under `src/`.

Core algorithm conventions to honor in forthcoming headers:
- Use only the `translation.x`, `translation.y`, and `translation.z` entries from the pose JSON to derive reference point C.
- Order candidate seeds by their Euclidean distance to C prior to running seeded DBSCAN.
- Run seeded DBSCAN with pure Euclidean metrics (`||p_i - p_j|| <= eps`) and return solely the cluster that contains the current seed.
- Validate clusters via minimum/maximum point counts and bounding diameter before accepting the first qualifying cluster.
- Optionally support voxel downsampling through a `voxel` parameter, using `pcl::VoxelGrid` when enabled.

The header files added in this stage are placeholders only; implementations and full interfaces will be introduced in later milestones.
