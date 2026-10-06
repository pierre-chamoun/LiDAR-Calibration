# LiDAR-Calibration

Extrinsic calibration between two LiDARs from a ROS 2 bag. Each mini-batch of scans is solved
as a GTSAM factor graph (scan-to-scan + cross-LiDAR GICP factors), its uncertainty is estimated
by Monte Carlo bootstrap, and the batches are fused recursively into a single estimate with a
covariance.

## 1. Build the Docker environment

Everything (ROS 2 Jazzy, PCL, GTSAM 4.2) is set up by the Docker image. From the repo root:

```bash
cd docker
docker compose up -d --build
docker compose exec lidar_calibration bash
```

The compose file mounts:

| Host                      | Container            |
| ------------------------- | -------------------- |
| this repository           | `~/ros_ws/src`       |
| `~/projects/datasets`     | `~/datasets`         |

Put your bags under `~/projects/datasets` on the host (or edit the volume in
[docker/docker-compose.yaml](docker/docker-compose.yaml)) so the container can see them.

## 2. Build the package

Inside the container:

```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

## 3. Run the calibration

```bash
ros2 launch lidar_calibration calibration.launch.xml \
    bag_path:=<path/to/bag> \
    config:=<path/to/config.yaml> \
    source_topic:=<reference_lidar_topic> \
    target_topic:=<lidar_to_calibrate_topic> \
    output_dir:=<path/to/output_dir> \
    start_offset_scans:=<num_scans_to_skip>
```

- `bag_path`: the ROS 2 bag to calibrate from (required).
- `config`: parameter file (default: the package's `config/config.yaml`).
- `source_topic`: reference LiDAR, `sensor_msgs/PointCloud2` (default: `/ouster0`).
- `target_topic`: LiDAR being calibrated, `sensor_msgs/PointCloud2` (default: `/ouster1`).
- `output_dir`: where results are written (default: `output/calibration`, relative to the
  current directory).
- `start_offset_scans`: reference scans to skip at the start of the bag (default: `0`).

### Configuration

The parameter files live in [lidar_calibration/config](lidar_calibration/config).

### Output

In `output_dir`:

- `calibration.csv`: one row per batch (estimate, 3σ bounds, twists).
- `recursive_estimate_<name>.csv`: the fused estimate and covariance after each batch.
- `fused_result.yaml`: the final fused extrinsic and its 3σ bounds.

## 4. Plot the recursive estimate

[analysis/plot_recursive_estimates.py](analysis/plot_recursive_estimates.py) finds every
`recursive_estimate_*.csv` under a directory and plots how each dataset's estimate and 3σ bound
evolve with the number of batches, one curve per dataset.

It needs `numpy`, `scipy`, `pandas`, `matplotlib` and a LaTeX installation (the figure uses
LaTeX text rendering). Run it from the repo root, either on the host or in the container:

```bash
python3 analysis/plot_recursive_estimates.py output --out output/recursive_estimates.pdf
```

The estimates are plotted as offsets from a common linearization point, by default the
information-weighted mean of every dataset's final estimate. To use a fixed point instead:

```bash
python3 analysis/plot_recursive_estimates.py output --lin 0 0 0 1.25 0.06 0
```

(`--lin ROLL_DEG PITCH_DEG YAW_DEG TX TY TZ`)
