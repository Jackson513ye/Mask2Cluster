# `apps`

Command-line entry points for mask2cluster will live here. A future `mask2cluster` CLI will parse runtime arguments then invoke the pipeline. This milestone intentionally ships no executable sources.

Planned interface for `apps/mask2cluster.cpp`:
- `--in <path>` — masked point cloud input (prefers `.las`, falls back to `.ply/.pcd`).
- `--pose <pose.json>` — pose metadata; only `translation` is consumed.
- `--out <cluster.ply>` — path for writing the accepted cluster.
- `--config <default.yaml>` — optional configuration file override.
- `--eps`, `--minPtsCore`, `--minPtsTotal`, `--maxDiameter`, `--maxTrials`, `--voxel` — optional overrides mirroring configuration parameters.

Implementation will arrive in later milestones once the pipeline solidifies.
