# mask2cluster

## Overview

`mask2cluster` receives a masked point cloud (point cloud 1) and a pose JSON, then searches **only inside that masked cloud** for the earliest acceptable cluster nearest to the reference point C. C is obtained from the pose JSON's `translation` component, and the clustering relies purely on Euclidean geometry without camera-specific context.

This repository currently ships only the project skeleton. No algorithms or compilable sources are implemented yet.

## Input / Output

- `--in <path.las>`: masked point cloud 1. `.ply` and `.pcd` are temporarily acceptable fallbacks when LAS support is unavailable.
- `--pose <pose.json>`: pose file; only `translation.x/y/z` are used to derive reference point C.
- `--out <cluster.ply>`: writes the first qualifying cluster found by the seeded DBSCAN search.

## Workflow Summary

1. Load point cloud 1 (prefer LAS via PDAL; allow `.ply/.pcd` when necessary) and parse C from the pose JSON.
2. Rank all points by ascending Euclidean distance to C and pick the nearest unbanned point as the seed.
3. Run a Euclidean seeded DBSCAN expansion from that seed, returning the cluster containing the seed.
4. Validate the cluster; if it fails thresholds, ban its points as future seeds and iterate with the next candidate until a cluster passes.
5. Export the first cluster that satisfies the validation rules as a `.ply` point cloud.

## Non-goals (Current Phase)

- Implementing clustering, IO, or math routines.
- Shipping any compiled binaries or command-line tools.
- Handling camera-specific parameters (normals, projections, intrinsics, etc.).

## Dependencies (Planned)

- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [Eigen](https://eigen.tuxfamily.org/)
- [PDAL](https://pdal.io/) for LAS ingestion (optional but preferred)

## Directory Layout

- `CMakeLists.txt` – minimal CMake skeleton; TODOs note future dependency discovery.
- `include/m2c/` – future header-only interfaces (currently placeholders with TODOs).
- `src/` – will hold C++ implementations; empty aside from documentation.
- `apps/` – will contain command-line entry points (no executables yet).
- `scripts/` – reserved for helper scripts.
- `data/` – sample pose and point cloud assets alongside default configuration stubs.

## Core Algorithm Conventions

- Reference point C comes strictly from `pose.json` `translation.x/y/z` values.
- Candidate seeds are pre-sorted by Euclidean distance to C.
- Seeded DBSCAN uses only Euclidean distance (`||p_i - p_j|| ≤ eps`) and returns the cluster containing the active seed.
- A cluster is accepted when `|S| ≥ minPts_total` and `diameter(S) ≤ maxDiameter`; otherwise every point in the failed cluster is banned from future seeding attempts (up to `max_trials`).
- Optional voxel downsampling leverages `pcl::VoxelGrid` when `voxel > 0`; downsampled clusters may be exported directly.

## Configuration Stub

Editable defaults live in `data/configs/default.ymal` with placeholders for the following parameters:
- `eps`
- `minPts_core`
- `minPts_total`
- `maxDiameter`
- `maxPts`
- `max_trials`
- `voxel`

## Next Steps

1. Flesh out header-only interface declarations under `include/m2c/`.
2. Implement module-by-module functionality within `src/` (IO, KDTree helpers, seeded DBSCAN, validation, orchestration pipeline).
3. Wire up a CLI entry point under `apps/` to parse runtime arguments and trigger the pipeline.

> **Note:** This milestone intentionally includes only repository scaffolding and descriptive documentation. No algorithmic implementation is provided yet.