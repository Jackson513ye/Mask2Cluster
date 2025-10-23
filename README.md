# mask2cluster

## Overview

`mask2cluster` receives a masked point cloud (point cloud 1) and a pose JSON, then searches **only inside that masked cloud** for the cluster most likely associated to the reference point C. C is obtained from the pose JSON's `translation` component, and the clustering relies purely on Euclidean geometry (camera-agnostic).

The project provides a command-line workflow that loads the masked cloud, parses pose metadata, performs FEC (Fast Euclidean Clustering), and exports the selected cluster as a `.ply` point cloud.

## Input / Output

- `--in <path.las>`: masked point cloud 1. The loader prefers `.las` when PDAL is enabled; if unavailable it falls back to `.ply`/`.pcd` via PCL IO.
- `--pose <pose.json>`: pose file; only `translation.x/y/z` are used to derive reference point C.
- `--out <cluster.ply>`: writes the selected cluster determined by the FEC-based pipeline.

## Workflow Summary

1. Load point cloud 1 (prefer LAS via PDAL; allow `.ply/.pcd` when necessary) and parse C from the pose JSON.
2. Run FEC (Fast Euclidean Clustering) using `eps` as the Euclidean tolerance.
3. Compute the mean cluster size `k` across all FEC labels and discard clusters smaller than `floor(n * k)`.
4. Consider all remaining clusters’ points together; take the `m` nearest points to C and select the cluster that appears most frequently among them (break ties by total distance to C).
5. Validate the selected cluster (size and diameter) and export it as a `.ply` point cloud.

## Non-goals (Current Phase)

- Recovering camera intrinsics/extrinsics or deriving view-dependent metrics.
- Reprojecting results back to the original full-scene point cloud.
- Performing advanced post-processing beyond seeded DBSCAN + validator checks.

## Dependencies

- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [Eigen](https://eigen.tuxfamily.org/)
- [PDAL](https://pdal.io/) for LAS ingestion (optional but preferred)
- [nlohmann/json](https://github.com/nlohmann/json) header-only parser for `pose.json`
- Lightweight in-repo YAML reader for configuration files (no external dependency)

## Build

```bash
cmake -S . -B build \
	-DM2C_ENABLE_BUILD=ON \
	-DM2C_WITH_PDAL=ON \
	-DM2C_BUILD_TOOLS=OFF

cmake --build build
```

Toggle flags:
- `M2C_WITH_PDAL=ON` (default) enables LAS ingestion; switch to `OFF` when PDAL is unavailable or unnecessary.
- `M2C_BUILD_TOOLS=ON` additionally builds the helper utilities `loader_probe` and `kd_probe`.

## Directory Layout

- `CMakeLists.txt` – top-level build toggles (`M2C_ENABLE_BUILD`, `M2C_WITH_PDAL`, `M2C_BUILD_TOOLS`).
- `include/m2c/` – public headers describing IO, KD-tree, validator, and pipeline interfaces.
- `src/` – implementations for pose/cloud IO, KD-tree wrapper, validator, and the FEC-based orchestration pipeline.
- `apps/` – CLI utilities (`mask2cluster`, plus development probes gated behind `M2C_BUILD_TOOLS`).
- `scripts/` – reserved for helper scripts.
- `data/` – sample pose/point cloud pairs and default configuration templates.
- `third_party/` – lightweight header shims (currently a minimal `nlohmann::json` implementation).

## Core Algorithm Conventions

- Reference point C comes strictly from `pose.json` `translation.x/y/z` values.
- First, run FEC clustering over the entire (masked) input cloud using a Euclidean tolerance (we reuse `eps` as the FEC radius).
- Compute the mean cluster size `k` across all FEC labels, then filter out clusters smaller than `floor(n * k)` where `n` is a fraction from config.
- Among the remaining clusters’ points, collect the `m` points nearest to C (Euclidean). The cluster that appears most among these `m` points is selected as the final result (ties broken by smaller total distance to C).
- The cluster diameter is estimated via an axis-aligned bounding box; final validation applies `minPts_total` (size) and `maxDiameter` (shape) where applicable.
- Optional voxel downsampling leverages `pcl::VoxelGrid` when `voxel > 0`; downsampled clusters may be exported directly.

## Configuration

Defaults live in `data/configs/default.yaml`. Runtime precedence is: CLI flags → YAML configuration → compiled-in defaults.

Key parameters:

- eps: Euclidean tolerance used by FEC. Larger merges more points; smaller splits clusters.
- n: Dynamic size filter factor. With `k = mean cluster size`, discard clusters with size < floor(n * k).
- m: Voting sample size near the reference point C. Among all kept clusters’ points, pick the cluster most frequent within the m nearest-to-C points.
- minPts_total: Minimum accepted cluster size at the final validation stage.
- maxDiameter: Maximum allowed diameter (AABB-based) for the selected cluster.
- voxel: Optional voxel downsampling leaf size (0 disables).
- minPts_core, maxPts, max_trials: legacy settings from seeded-DBSCAN; retained for compatibility but unused by the FEC pipeline.

## Usage

After building, invoke the CLI with masked cloud, pose JSON, and output path. Configuration values are sourced from compiled defaults, optionally overridden by a YAML file, and finally by CLI flags.

```bash
./build/mask2cluster \
	--in data/example_maskpoint.las \
	--pose data/example_position.json \
	--out output/cluster.ply \
	--config data/configs/default.yaml \
	--maxDiameter 10.0 \
	--maxTrials 150 \
	--voxel 0.03
```

Key flags:
- `--in`, `--pose`, `--out` – required inputs (LAS preferred when PDAL is available).
- `--config` – optional YAML file mirroring `data/configs/default.yaml`.
- `--eps`, `--minPtsCore`, `--minPtsTotal`, `--maxDiameter`, `--maxPts`, `--maxTrials`, `--voxel`, `--n`, `--m` – override parameters directly from the command line.
 - The sample dataset may require relaxing `maxDiameter` (for instance `--maxDiameter 10.0`) to surface a qualifying cluster.