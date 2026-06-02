# Trajectory Evaluation with evo

For Week 4 evaluation against RTK GNSS ground truth.
The `evo` toolkit reads ROS2 bags directly and computes ATE and RPE.

## Installation

```bash
pip install evo --upgrade
```

## Concepts

- **ATE (Absolute Trajectory Error)**: root-mean-square distance between
  estimated and ground-truth poses after optimal alignment. Global accuracy.
- **RPE (Relative Pose Error)**: pose drift over fixed time or distance windows.
  Local/incremental accuracy — better reflects what you'd see during a walk.

---

## Workflow

### Step 1 — Prepare Ground Truth

The RTK GNSS trajectory from `/gnss/fix` (NavSatFix messages) must be
converted to a TUM-format file or kept as a ROS2 bag topic.

The `evo` toolkit can read NavSatFix directly from a ROS2 bag if you convert
it to a path first. Use this helper script:

```bash
# Convert NavSatFix bag to TUM trajectory file
# (LLH → local ENU, origin = first fix)
python3 evaluation/navsat_to_tum.py \
  --bag /data/bags/mmb_20260518_0930_courtyard_walk_01 \
  --topic /gnss/fix \
  --output /data/bags/ground_truth_walk_01.tum
```

*(See `evaluation/navsat_to_tum.py` — to be created after GNSS integration is complete.)*

### Step 2 — Extract Estimated Trajectory

If you ran a SLAM or odometry stack on the bag, its output is typically a
`nav_msgs/Odometry` or `geometry_msgs/PoseStamped` topic. Extract it:

```bash
evo_traj bag2 /data/bags/mmb_20260518_0930_courtyard_walk_01 \
  /odom --save_as_tum -o /data/bags/estimated_walk_01.tum
```

### Step 3 — Compute ATE

```bash
evo_ape tum \
  /data/bags/ground_truth_walk_01.tum \
  /data/bags/estimated_walk_01.tum \
  --align --correct_scale \
  --plot --plot_mode xy \
  --save_results /data/results/ape_walk_01.zip
```

### Step 4 — Compute RPE

```bash
evo_rpe tum \
  /data/bags/ground_truth_walk_01.tum \
  /data/bags/estimated_walk_01.tum \
  --align --delta 1 --delta_unit m \
  --plot --plot_mode xy \
  --save_results /data/results/rpe_walk_01.zip
```

### Step 5 — Compare Multiple Runs

```bash
evo_res /data/results/*.zip --use_filenames --save_table results_summary.csv
```

---

## Sanity Checks Before Running evo

```bash
# Verify both trajectories have overlapping timestamps
evo_traj tum ground_truth_walk_01.tum estimated_walk_01.tum --plot

# The two tracks should overlap spatially after alignment.
# If they don't, check:
#   1. Timestamp units (evo expects seconds, not nanoseconds)
#   2. Coordinate frame consistency (both must be in the same world frame)
#   3. Time sync — was chrony running during the recording?
```

---

## Reporting Metrics

Report both ATE and RPE for each test run. Typical acceptable values for
a walking-speed backpack system:

| Metric | Good | Acceptable |
|--------|------|------------|
| ATE RMSE | < 0.1 m | < 0.5 m |
| RPE (1 m window) | < 0.05 m | < 0.2 m |

These thresholds depend heavily on which estimation pipeline is used.
Document the pipeline version alongside every result.
