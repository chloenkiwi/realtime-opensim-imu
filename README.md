# Real-Time OpenSim via IMUs

Research code accompanying the IEEE TNSRE article **“Real-Time OpenSim via IMUs for Full Body Kinematics During Gait, Sports, Exercise, and Dance Movements.”**

**DOI:** [10.1109/TNSRE.2026.3653477](https://doi.org/10.1109/TNSRE.2026.3653477)

This repository estimates full-body joint kinematics from streamed inertial measurement unit (IMU) orientations. It receives quaternion data from a SageMotion system, calibrates an OpenSim musculoskeletal model, and updates a real-time inverse-kinematics solver. Recorded IMU data can also be processed in offline mode.

> **Release status:** research prototype. Complete the items in [Known limitations](#known-limitations) before creating an archival release.

## Features

- Real-time quaternion streaming over WebSocket
- IMU-to-OpenSim orientation conversion
- Static calibration with optional quaternion averaging
- Real-time inverse kinematics using a modified OpenSim build
- Optional OpenSim visualization
- Offline processing of recorded SageMotion CSV data
- Joint-angle and timing output in CSV format
- Optional compensation for dropped sensor packets

## Repository contents

```text
.
├── ik_streaming.py              # Command-line entry point and IK loop
├── config_manager.py            # TOML configuration loader
├── data_stream_client.py        # SageMotion WebSocket client
├── helper.py                    # File conversion and quaternion utilities
├── config.toml                  # Example/default configuration
├── requirements.txt             # Python dependencies
├── clamped_Rajagopal_2015.osim # Musculoskeletal model used by default
└── Geometry/                    # OpenSim visualization meshes
```

`temp_file.sto` is a runtime interchange file written by the application. It should not be treated as input data or included in an archival release.

## Requirements

The currently tested software configuration is:

- Windows
- Python 3.10
- The project-specific OpenSim 4.5.1-alpha fork
- SageMotion IMUs and an application that exposes the WebSocket stream for real-time use

The Python dependencies listed in `requirements.txt` include NumPy, SciPy, transforms3d, websocket-client, tomli, and ntplib.

## Installation

### 1. Create a Python environment

```bash
conda create -n opensim-imu python=3.10
conda activate opensim-imu
```

### 2. Clone this repository and install its dependencies

```bash
git clone https://github.com/RTnhN/OpenSimRT.git
cd OpenSimRT
python -m pip install -r requirements.txt
```

### 3. Install the modified OpenSim build

Download the OpenSim binaries from the [OpenSim 4.5.1-alpha release](https://github.com/RTnhN/opensim-core/releases/tag/4.5.1-alpha) in the project fork. If the archive is extracted to `C:\opensim-core`, install its Python package with:

```bash
cd C:\opensim-core\sdk\python
python -m pip install .
```

Verify the installation from any directory:

```bash
python -c "import opensim; print(opensim.__version__)"
```

The real-time solver uses `InverseKinematicsSolverRT`, which is provided by the modified OpenSim build and may not be available in the standard OpenSim Python package.

## Sensor configuration

Edit `config.toml` before starting the application. The order of entries in `sensors` **must exactly match** the sensor order in each streamed packet or offline CSV file.

The default configuration uses 12 IMUs:

```toml
sensors = [
  "calcn_l_imu", "calcn_r_imu",
  "tibia_l_imu", "tibia_r_imu",
  "femur_l_imu", "femur_r_imu",
  "pelvis_imu", "torso_imu",
  "radius_l_imu", "radius_r_imu",
  "humerus_l_imu", "humerus_r_imu"
]
base_imu = "pelvis_imu"
base_imu_axis = "-z"
```

Important options include:

| Option | Meaning |
| --- | --- |
| `offline` | Read a recorded CSV instead of the live WebSocket stream |
| `offline_data_name` | CSV filename inside the `offline/` directory |
| `rate` | Requested IK output rate in hertz |
| `ave_quat` | Average an initial two-second calibration window |
| `visualize` | Display the OpenSim visualizer |
| `log_angle` | Save estimated joint kinematics |
| `log_time` | Save timing and end-to-end delay information in online mode |
| `datadrop_comp` | Enable dropped-packet compensation |
| `clamped` | Select the clamped Rajagopal model |

## Real-time use

1. Attach and label the IMUs according to the experimental protocol.
2. Start the SageMotion application and enable quaternion streaming.
3. Keep the participant stationary in the calibration pose.
4. Start the IK application with the streaming server address:

```bash
python ik_streaming.py --address 192.168.137.1
```

By default, the program skips 100 incoming samples and then averages the next 200 samples for calibration. `--start` is a **sample count**, not a duration in milliseconds. For example:

```bash
python ik_streaming.py --address 192.168.137.1 --start 200
```

Press `Ctrl+C` to stop processing.

## Offline use

Set the following values in `config.toml`:

```toml
offline = true
offline_data_name = "example.csv"
```

Place the recording at `offline/example.csv`, then run:

```bash
python ik_streaming.py
```

When `ave_quat = true`, an offline recording must contain at least the number of samples skipped by `--start` plus 200 calibration samples and additional samples for IK. The CSV columns must use the SageMotion naming convention, including `Quat1_1` through `Quat4_N` for sensors 1 through N. If available, `Package_N` is used as the packet counter.

## Coordinate and quaternion conventions

The application applies a -90-degree rotation about the z axis to convert VRU-mode orientations to the y-up convention expected by this model. Before using the code with another IMU system, verify:

- the order represented by `Quat1`, `Quat2`, `Quat3`, and `Quat4`;
- whether the quaternion is scalar-first or scalar-last;
- the direction of the sensor-to-world rotation;
- the sensor mounting axes; and
- the world and OpenSim coordinate-system handedness.

These conventions affect every estimated joint angle and should be reported with experimental results.

## Outputs

When enabled, the application creates:

- `output.csv`: time and OpenSim coordinate values;
- `time.csv`: sample time, source timestamp, IK computation time, and estimated end-to-end delay; and
- `calibrated_*.osim`: the calibrated musculoskeletal model.

OpenSim coordinate values are stored in the units returned by the model. Check the model coordinate definitions before interpreting or converting them.

## Attribution

This work was inspired by Patrick Slade's [RealTimeKin](https://github.com/pslade2/RealTimeKin) project.

The included musculoskeletal model is derived from the Rajagopal full-body model. Cite the original model publication and OpenSim in any resulting work, and verify the redistribution terms for the model and geometry assets.

## Citation

If you use this code, please cite:

> C. Xu et al., "Real-Time OpenSim via IMUs for Full Body Kinematics During Gait, Sports, Exercise, and Dance Movements," *IEEE Transactions on Neural Systems and Rehabilitation Engineering*, vol. 34, pp. 650–662, 2026, doi: [10.1109/TNSRE.2026.3653477](https://doi.org/10.1109/TNSRE.2026.3653477).

**Keywords:** Kinematics; real-time systems; calibration; humanities; estimation; biomedical optical imaging; accuracy; tracking; stairs; optical fiber sensors; biomechanics; inertial sensors.
