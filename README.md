[readme.md](https://github.com/user-attachments/files/28135257/readme.md)
# Magnetic Navigation Simulation (MagNavSim)

A plugin and message package for gz‑sim (Gazebo Sim) that simulates magnetic anomalies around an AUV, publishes both magnetometer field data and converted frequency (Hz) messages, and logs / visualises field magnitude vs AUV position.

## Prerequisites

- A supported Linux distribution (Ubuntu 22.04 recommended)
- Gazebo Harmonic. [Link to install](https://gazebosim.org/docs/harmonic/install_ubuntu/)  
- A ROS 2 / ament / colcon workspace for building (since we use `colcon build`).  
- Dependencies listed in `package.xml`: `gz-common5`, `gz-msgs10`, `gz-plugin2`, `gz-sim8`, `gz-transport13`.

## Installation & Build

1. Create a ROS2 workspace
  ```bash
  mkdir -p ~/ros2_ws/src
  ```

2. Clone or place the repository inside `src` folder.

3. Build the package.
  ```bash
  cd ~/ros2_ws
  colcon build --symlink-install
  ```

4. Once built, source the install space:
  ```bash
  source install/setup.bash
  ```

5. Set up environment variables so that Gazebo Sim can locate models, worlds, plugins and message descriptors:
  ```bash
  export GZ_SIM_RESOURCE_PATH=/home/ubuntu/Desktop/ros2_ws/src/MagNavSim/magnetic_navigation_sim/models:$GZ_SIM_RESOURCE_PATH
  export GZ_PLUGIN_PATH=/home/ubuntu/Desktop/ros2_ws/install/magnetic_navigation_sim/lib:$GZ_PLUGIN_PATH
  export GZ_DESCRIPTOR_PATH=/home/ubuntu/Desktop/ros2_ws/install/magnetic_navigation_sim/share/gz/protos:$GZ_DESCRIPTOR_PATH
  ```

(Adjust paths to workspace as needed.)

## Running the Simulation

To run the simulation:
```bash
gz sim magnetic_navigation_sim/worlds/water_world.sdf --verbose
```

## What to Expect
The plugin will publish custom messages that include:
  - The AUV's magnetometer vector (`gz::msgs::Magnetometer`)
  - The computed field magnitude in nT
  - The derived frequency in Hz

To inspect custom message:
```bash
gz topic -l
gz topic -e -t /tethys/magfreq -m gz.magnetic_nav_msgs.MagFreq
```

The gz logs will contain lines like:
```bash
[MagAnomalySystem] field_nT=50084.6, freq_Hz=261156284.6
```

## Reading & Plotting the Messages

```bash
gz sim magnetic_navigation_sim/worlds/water_world.sdf --verbose 2>&1 \
  | sed -r "s/\x1B\[[0-9;]*[mK]//g" \
  | tee sim.log
```

Then extract the logged data into CSV:
```bash
grep "\[MagAnomalySystem\] LOG" sim.log \
  | sed 's/.*x_pos(m)=$begin:math:text$[^,]*$end:math:text$, B_mag(nT)=$begin:math:text$[^ ]*$end:math:text$.*/\1,\2/' \
  > pos_vs_field.csv
```

## Plugin Configuration

### Magnetic Anomaly Plugin  
**`magnetic_navigation_sim::MagAnomalySystem`**

This plugin simulates **spatially varying magnetic fields** in the Gazebo world and publishes a combined **total-field magnitude** and **frequency-based magnetometer signal**.
Instead of analytical dipole models, the magnetic environment is represented using a **precomputed, data-driven magnetic field grid**, enabling realistic and smooth magnetic behavior derived from real measurements.

The plugin is intended to provide reliable sensor input for **magnetic navigation, avoidance, and control algorithms**.

#### Published Topic

- **Message type:** `gz::magnetic_nav_msgs::MagFreq`  
- **Contents:** total magnetic field magnitude (nT), mapped frequency (Hz), and magnetic field vector (for simulation/debug use)

#### Magnetic Field Model (Grid-Based)
The magnetic environment is represented as a **2D spatial grid** of magnetic field magnitudes:
- The grid is generated offline from logged trajectory data
- Magnetic field values are expressed in **nanotesla (nT)**
- The field varies over the horizontal (XY) plane
- The Z-component is assumed constant (surface-field approximation)

At runtime, the plugin:
1. Queries the grid using the AUV sensor’s world position
2. Applies **bilinear interpolation** to compute the local magnetic field magnitude
3. Combines this with the ambient Earth field

\[
B_total (x,y) = B_grid (x,y)
\]

#### Frequency Calibration Model
The magnetometer frequency output is computed using a **linear calibration equation**:

\[
\boxed{B = \frac{f - f_0}{S}}
\]

or equivalently:

\[
\boxed{f = f_0 + S \cdot B}
\]

Where:
- \(f_0\) is the baseline (offset) frequency in Hz  
- \(S\) is the sensitivity constant (Hz per nT)  
- \(B\) is the magnetic field magnitude in nT 

#### SDF Parameters

**Core Parameters**
| Parameter | Type | Description |
|----------|------|-------------|
| `auv_model_name` | string | Name of the AUV model in the world |
| `magnetometer_link_name` | string | Link on the AUV where the magnetometer sensor is mounted |
| `grid_file` | string (path) | Absolute path to binary magnetic field grid file |
| `update_rate` | double (Hz) | Rate at which magnetic field values are computed and published |
| `threshold_field_nT` | double | Reference magnetic field threshold (nT) |

**Magnetic Field Grid (grid_file)**

The grid file contains a precomputed magnetic field map generated offline from measured data.

Grid characteristics:
- Units: nanotesla (nT)
- Coordinate frame: Gazebo-aligned XY plane
- Resolution: fixed, defined during grid generation
- Z-distance: ignored (2D surface field)

> ⚠️ The grid is assumed to already be aligned and scaled to Gazebo coordinates.
> No additional coordinate transformation or resolution scaling is performed inside the plugin.

**Frequency Calibration Parameters**

| Parameter | Type | Description |
|----------|------|-------------|
| `frequency_baseline` | double (Hz) | Baseline sensor frequency \(f_0\) |
| `sensitivity_constant` | double (Hz/nT) | Sensor sensitivity \(S\) |

#### Example SDF Configuration

```xml
<!-- Magnetometer Anomaly Plugin -->
<plugin filename="libmagnetic_navigation_sim_anomaly.so"
        name="magnetic_navigation_sim::MagAnomalySystem">

  <auv_model_name>tethys_equipped</auv_model_name>
  <magnetometer_link_name>mag_link</magnetometer_link_name>

  <!-- Precomputed magnetic field grid -->
  <grid_file>
    /home/ubuntu/Desktop/ros2_ws/src/MagNavSim/data/magnetic_field_grid.bin
  </grid_file>

  <update_rate>20</update_rate>

  <!-- Linear frequency calibration -->
  <frequency_baseline>2.50e8</frequency_baseline>
  <sensitivity_constant>255</sensitivity_constant>

  <threshold_field_nT>60000</threshold_field_nT>
</plugin>
```

### Magnetic Navigation Controller Plugin  
**`magnetic_navigation_sim::MagNavControllerSystem`**

This plugin implements a **deterministic, time-based magnetic navigation and avoidance controller** for an AUV.

The controller consumes magnetometer data for situational awareness but executes obstacle avoidance using **predefined, time-parameterized motion primitives**, rather than reacting directly to magnetic magnitude or statistical thresholds. This design works reliably with **grid-based magnetic field models**.

The controller operates as a finite-state machine with the following high-level states:

- **TRACK** – nominal straight-line motion
- **AVOID** – magnetic anomaly avoidance using magnitude rise/peak/fall
- **REJOIN** – return to nominal tracking after clearing the obstacle

Arc direction (left/right) is determined **deterministically from the obstacle’s world Y coordinate**, which is resolved internally by the plugin using the obstacle name provided in the avoidance profiles.

#### Subscribed / Published Topics

- **Subscribed**
  - Magnetometer topic publishing `gz::magnetic_nav_msgs::MagFreq`

- **Published**
  - Fin command topic (`gz::msgs::Double`)
  - Propeller thrust command topic (`gz::msgs::Double`)

#### Time-Based Avoidance Model

Each avoidance maneuver is divided into three explicit time phases:

| Phase | Time Interval | Fin Command |
|---------|------|-------------|
| Phase 1 | `avoid_start_time` → `avoid_mid_time`
 | Arc outward |
| Phase 2 | `avoid_mid_time` → `avoid_end_time`
 | Counter-arc |
| Phase 3 | End of phase 2 → completion
 | Arc outward (re-alignment) |

#### SDF Parameters (Single Reference Table)

| Parameter | Type | Description |
|---------|------|-------------|
| `auv_model_name` | string | Name of the AUV model in the world |
| `sensor_topic` | string | Topic from which magnetic sensor data (`MagFreq`) is received |
| `fin_cmd_topic` | string | Topic used to command fin position |
| `prop_cmd_topic` | string | Topic used to command propeller thrust |
| `threshold_field_nT` | double | Magnetic field magnitude threshold (nT) used to trigger avoidance |
| `threshold_field_std` | double | Standard deviation threshold used to detect magnetic anomalies |
| `field_std_window` | int | Window size for rolling magnetic statistics |
| `field_ema_alpha` | double | Exponential moving average smoothing factor for magnetic field |
| `field_slope_epsilon` | double | Minimum slope magnitude to classify rising or falling field trends |
| `forward_thrust` | double | Nominal forward thrust command during tracking |
| `max_fin_pos` | double (rad) | Maximum allowable fin deflection |
| `fin_gain` | double | Proportional gain for heading correction during tracking |
| `min_state_duration_s` | double | Minimum time the controller must remain in each FSM state |
| `avoid_profiles` | block | List of obstacle-specific avoidance profiles (see below) |

#### Avoidance Profiles

Each entry in `avoid_profiles` defines how a **specific magnetic obstacle** should be avoided.
The controller processes profiles sequentially and uses the obstacle’s world Y coordinate to decide the arc side.

##### Avoid Profile Fields

| Field | Type | Description |
|------|------|-------------|
| `obstacle_name` | string | Name of the obstacle entity in the world |
| `avoid_start_time` | double | Start time of Phase 1 |
| `avoid_mid_time` | double | Transition time between Phase 1 and Phase 2 |
| `avoid_end_time` | double | End time of Phase 3 (avoidance complete) |

#### Example SDF Configuration

```xml
<!-- AUV Controller Plugin -->
<plugin filename="libmagnetic_navigation_sim_controller.so"
        name="magnetic_navigation_sim::MagNavControllerSystem">

  <auv_model_name>tethys_equipped</auv_model_name>

  <sensor_topic>/tethys/magfreq</sensor_topic>
  <fin_cmd_topic>/model/tethys_equipped/joint/vertical_fins_joint/cmd_pos</fin_cmd_topic>
  <prop_cmd_topic>/model/tethys_equipped/joint/propeller_joint/cmd_thrust</prop_cmd_topic>

  <forward_thrust>-0.3</forward_thrust>
  <max_fin_pos>0.261799</max_fin_pos>
  <fin_gain>5.0</fin_gain>
  <min_state_duration_s>5.0</min_state_duration_s>

  <avoid_profiles>
    <profile>
      <obstacle_name>mag_source_jay</obstacle_name>
      <avoid_start_time>15.0</avoid_start_time>
      <avoid_mid_time>20.0</avoid_mid_time>
      <avoid_end_time>22.0</avoid_end_time>
    </profile>

    <profile>
      <obstacle_name>mag_source_tracey</obstacle_name>
      <avoid_start_time>25.0</avoid_start_time>
      <avoid_mid_time>130.0</avoid_mid_time>
      <avoid_end_time>170.0</avoid_end_time>
    </profile>

    <profile>
      <obstacle_name>mag_source_merci</obstacle_name>
      <avoid_start_time>1.0</avoid_start_time>
      <avoid_mid_time>2.0</avoid_mid_time>
      <avoid_end_time>3.0</avoid_end_time>
    </profile>
  </avoid_profiles>
</plugin>
```

## Key Design Notes
- Avoidance behavior is **fully time-driven**
- Arc side selection is deterministic and computed using obstacle world Y position
- No AUV world position is used in control laws
- Profiles allow per-obstacle tuning for size, strength, and clearance

## Island Constant Determination

To achieve realistic and location-specific magnetic behavior, **per-island magnetic anomaly constants** were derived directly from real AUV trajectory data and logged magnetometer measurements.

- `data/mag_data.csv`: This file contains the AUV trajectory (UTM coordinates) along with measured magnetic field values collected during the mission.

- `data/plot_islands.py`: This script plots the AUV trajectory together with the inferred island locations.

- `data/build_grid_magnetic_map.py`: This script converts trajectory-aligned magnetic measurements into Gazebo-compatible coordinates and produces a magnetic field grid file used directly by the simulation plugin.

- `data/plot_magnetic_gradients.py`: This script computes the spatial gradients of the magnetic field and visualizes them.

The resulting grid preserves:
- True measured field magnitudes
- Realistic spatial gradients
- Consistency with Gazebo’s coordinate frame

### Validation via Query Plugin

To validate the tuned constants independently of the offline fitting pipeline:

- A **QueryPlugin** was implemented (available in the `data-tuning` branch).
- This plugin allows querying the magnetic field at arbitrary positions in the simulation without running the full AUV mission.
- Validation can be performed by launching:

## Results
![Plot](images/sim_data.png)
*Tuning Plot*

![Simulation Configuration](images/Plot.png)
*Simulation Configuration*

![Magnetic Grid Heatmap](images/heatmap.png)
*Magnetic Field Magnitude Heatmap*
