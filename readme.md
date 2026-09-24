# RACE AUV

## Introduction
This is the configuration for the RACE AUV on the ROS2-MVP framework.
- ROS2 version Jazzy
- Ubuntu 24.04

## Directory structure
- `race_auv`: Meta package for the RACE AUV.

- `race_auv_bringup`: Launch files & configurations to bring the vehicle/simulation up and running.
    - `launch/bringup_simulation.launch.py` is the main launch file for the simulation.
    - `launch/include` holds the sub-launch files called by the bringup files (`launch/include/simulation` for the simulation-only ones).
    - `config` holds the ROS parameter `*.yaml` files loaded by the sub-launch files.

- `race_auv_config`: MVP configuration files for the controller (`mvp_control_config`) and helm (`mvp_mission_config`).

- `race_auv_description`: URDF files, rviz configuration, and vehicle mesh.

- `race_auv_perception` (**submodule**): `race_auv_camera_pkg` + `race_auv_apriltag_cuda`, plus the `third_party/isaac_ros_nitros` cuAprilTags submodule.

- `race_auv_sim` (**submodule**): `race_auv_sim_pkg`.

See [LOCALIZATION_ARCHITECTURE.md](LOCALIZATION_ARCHITECTURE.md) for how the localization stack is put together.

## Installation

### Install the Stonefish simulator (simulation only)
- We use [Stonefish](https://stonefish.readthedocs.io/en/latest/install.html) Simulator. You can clone it from [here](https://github.com/GSO-soslab/stonefish), a fork from the [original_repo](https://github.com/patrykcieslak/stonefish).

- Download the stonefish simulator **to another location outside your ROS workspace**
    ```bash
    git clone https://github.com/GSO-soslab/stonefish
    ```

- Install dependencies using `sudo apt install` (instruction from the [Stonefish](https://github.com/patrykcieslak/stonefish))
    * **OpenGL Mathematics library** (libglm-dev, version >= 0.9.9.0)
    * **SDL2 library** (libsdl2-dev, may need the following fix!)
        1. Install SDL2 library from the repository.
        2. `cd /usr/lib/x86_64-linux-gnu/cmake/SDL2/`
        3. `sudo vim sdl2-config.cmake`
        4. Remove space after "-lSDL2".
        5. Save file.
    * **Freetype library** (libfreetype6-dev)

- Build and install the stonefish
    ```bash
    cd stonefish
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make -j$(nproc)
    sudo make install
    ```

### Setup RACE AUV Repo
- Clone the `race_auv` repo into your workspace `src` folder. Use the `jazzy-devel-perception` branch (the default branch on GitHub is `noetic-devel`).
    ```bash
    cd ~/ros2_ws/src
    git clone --branch jazzy-devel-perception https://github.com/GSO-soslab/race_auv.git
    cd race_auv
    ```

- Perception and simulation packages live in their own repositories and are linked here as **git submodules**, so each machine only pulls what it runs. After cloning, the `race_auv_perception` and `race_auv_sim` folders are empty until you initialize them. Pick the row for your machine:

    | Machine | Init these submodules | Notes |
    | --- | --- | --- |
    | Frontseat Pi5 | none | No CUDA / perception / sim packages are built. |
    | Backseat Jetson | `race_auv_perception` | Then run `race_auv_perception/scripts/setup_third_party.sh` for the cuAprilTags LFS library. |
    | Sim computer | `race_auv_perception` `race_auv_sim` | No CUDA toolkit needed: `race_auv_apriltag_cuda` builds without the native shims and `apriltag_detector_node` uses the CPU backend. |

    ```bash
    # Backseat Jetson
    sudo apt install git-lfs
    git submodule update --init race_auv_perception
    race_auv_perception/scripts/setup_third_party.sh

    # Sim computer
    git submodule update --init race_auv_perception race_auv_sim
    ```

- **Do not** use `git clone --recursive` or `git submodule update --init --recursive`: the `isaac_ros_nitros` submodule would pull hundreds of MB of Git LFS objects. On the Jetson, `setup_third_party.sh` sparse-checks out only `lib/cuapriltags` and pulls just those blobs.

- Updating later: after `git pull`, run the same `git submodule update --init <submodules>` command again so the submodules move to the commits this branch points at.

- Install pip
    ```bash
    sudo apt install python3-pip
    ```

### Install ROS-MVP
Currently MVP packages should be built from source.
Target platform must be Ubuntu 24.04 because of the dependencies.

Pull the repositories into the same `src` folder (next to `race_auv`):
```bash
cd ~/ros2_ws/src
git clone --single-branch --branch jazzy-devel https://github.com/uri-ocean-robotics/mvp_msgs
git clone --single-branch --branch jazzy-devel https://github.com/uri-ocean-robotics/mvp_control
git clone --single-branch --branch jazzy-devel-teleop-twist https://github.com/uri-ocean-robotics/mvp_mission
git clone --single-branch --branch jazzy-devel https://github.com/uri-ocean-robotics/mvp_utilities.git
git clone --single-branch --branch jazzy-devel https://github.com/GSO-soslab/mvp_c2.git
git clone --single-branch --branch jazzy-devel https://github.com/GSO-soslab/acomms_msgs.git
```

- `mvp_mission` must be on `jazzy-devel-teleop-twist`: the RACE helm config uses the `teleop_twist` control mode.
- `mvp_c2` and `acomms_msgs` are used by the C2 (command & control) launch files.

For the simulation, also pull:
```bash
git clone https://github.com/GSO-soslab/stonefish_ros2.git
git clone --single-branch --branch jazzy-devel-race-docking-station https://github.com/GSO-soslab/world_of_stonefish
git clone --single-branch --branch jazzy-devel-docking-sim https://github.com/GSO-soslab/race_station.git
```

- **stonefish_ros2** is the ROS2 interface for Stonefish simulator.
- **world_of_stonefish** holds the scenario files (the RACE AUV uses `world/race_auv_test.scn`) and the drivers that turn Stonefish sensor messages into MVP-compatible messages. It must be on the `jazzy-devel-race-docking-station` branch, which has the RACE docking station, its AprilTags and lights in the scenario.
    - If you already have `world_of_stonefish` cloned on another branch, switch it:
        ```bash
        cd world_of_stonefish
        git fetch origin jazzy-devel-race-docking-station:jazzy-devel-race-docking-station
        git checkout jazzy-devel-race-docking-station
        ```
- **race_station** provides `race_station_description/urdf/base.urdf`, the docking-station tag layout that `apriltag_fuser_node` loads in simulation (`object.urdf_package` in `race_auv_bringup/config/simulation/apriltag.yaml`). The station's physics and visuals come from the `world_of_stonefish` scenario; the station's own bringup is commented out in `bringup_simulation.launch.py`.

### Install ROS dependencies
From the workspace root, let rosdep install the remaining apt packages (`robot_localization`, `joy`, `vision_msgs`, `cv_bridge`, `python3-scipy`, ...):
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys dwe_camera_driver
```
`dwe_camera_driver` is the camera driver for the real vehicle (Jetson) and is not needed in simulation. For Jetson setup see `race_auv_perception/race_auv_camera_pkg/JETSON_ORIN_NANO_SETUP.md`.

### Install apriltag3 (sim computer)
The sim computer does not need CUDA. `race_auv_apriltag_cuda` still builds (it just skips the CUDA shims with a warning), and `race_auv_bringup/config/simulation/apriltag.yaml` selects the CPU backends (`detector_backend: "python"`, `image_pipeline: "cpu"`).

That Python backend needs the upstream [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag) binding **with pose estimation**. rosdep does not install it, it is not on pip, and the Ubuntu `python3-apriltag` package lacks `estimate_tag_pose` (every detection would fall back to an identity pose). Build it from source **outside your ROS workspace**:
```bash
sudo apt install -y cmake build-essential python3-dev python3-numpy
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
cmake -B build -DCMAKE_BUILD_TYPE=Release
sudo cmake --build build --target install
sudo ldconfig

# Upstream installs the Python wrapper one directory deeper than libapriltag.so.3
sudo ln -sf ../../libapriltag.so.3 /usr/local/lib/python3.12/site-packages/libapriltag.so.3

python3 -c "from apriltag import apriltag; print('apriltag3 OK')"
```
See section 8.1 of `race_auv_perception/race_auv_camera_pkg/JETSON_ORIN_NANO_SETUP.md` for a no-root install.

### Compile the code
Go back to the ROS workspace dir (e.g., `ros2_ws`), then do
```bash
colcon build
source install/setup.bash
```

## Quick test
- Bring up the RACE AUV with the Stonefish simulator.
    ```bash
    ros2 launch race_auv_bringup bringup_simulation.launch.py
    ```
    RViz opens with `race_auv_description/rviz/config.rviz`, which shows:
    - the vehicle model and TF
    - the filtered odometry
    - the path-following path and segment markers
    - the AprilTag dock pose (`/race_station/dock_point/pose`)
    - the `cam_front` and `cam_down` images

    > If RViz dies with `symbol lookup error: /snap/core20/.../libpthread.so.0`, you launched from the terminal of a snap-installed VS Code. Launch from a normal terminal, or clear the snap variables first: `unset GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR GTK_IM_MODULE_FILE GDK_PIXBUF_MODULE_FILE GDK_PIXBUF_MODULEDIR LOCPATH GSETTINGS_SCHEMA_DIR`.

- Enable the controller in a separate terminal
    ```bash
    ros2 service call /race_auv/controller/set std_srvs/srv/SetBool "{data: true}"
    ```

- Start a path following mission in the local frame. The waypoints are defined under `bhv_path_following` in `race_auv_bringup/config/bhv_params_sim.yaml`.
    ```bash
    ros2 service call /race_auv/mvp_helm/change_state mvp_msgs/srv/ChangeState "{state: 'survey', caller: 'user'}"
    ```

- Teleoperate with a joystick (the `joy` node is launched with the simulation)
    ```bash
    ros2 service call /race_auv/mvp_helm/change_state mvp_msgs/srv/ChangeState "{state: 'teleop', caller: 'user'}"
    ```

- You can put the AUV in idle anytime by changing the state of the helm
    ```bash
    ros2 service call /race_auv/mvp_helm/change_state mvp_msgs/srv/ChangeState "{state: 'start', caller: 'user'}"
    ```

- The available helm states (`start`, `kill`, `teleop`, `teleop_twist`, `survey`, `mapping`) and their transitions are in `race_auv_config/mvp_mission_config/helm_sim.yaml`.

## Customizing the simulation
The simulation is split between the Stonefish scenario files in `world_of_stonefish` and the ROS-side config in `race_auv_bringup`.

> **Rebuild after editing.** Stonefish reads the scenario from the *installed* copy. After changing anything in `world_of_stonefish`, run `colcon build --packages-select world_of_stonefish` (or build with `--symlink-install`).

### Where things live

| To change... | Edit | Notes |
| --- | --- | --- |
| Which world file is loaded | `race_auv_bringup/launch/include/simulation/simulation.launch.py` | `sim_world = 'race_auv_test.scn'`. The simulation rate, window size and rendering quality are set in the same file. |
| Water density, turbidity (`jerlov`), temperature, waves, current, sun | `race_auv_bringup/config/sim_params.yaml`, under `/stonefish_simulator` | The world file reads these through `$(param ...)`. |
| Tank, seabed, and which vehicles are spawned | `world_of_stonefish/world/race_auv_test.scn` | Includes `vehicles/race_auv.scn` and `vehicles/race_station.scn`. Meshes are in `world_of_stonefish/data/objects/`. |
| AUV start pose | `world_of_stonefish/vehicles/race_auv.scn`, `<world_transform>` near the end of `<robot>` | NED: `z` is positive down. |
| Docking-station pose | `world_of_stonefish/vehicles/race_station.scn`, `<world_transform>` near the end | Older poses are left commented out above it. |
| AUV mass, CG, buoyancy | `world_of_stonefish/vehicles/race_auv.scn`, `<base_link name="Vehicle">` | See [Mass and buoyancy](#mass-and-buoyancy) below. |
| Thruster position, direction, thrust | `race_auv.scn`, `<actuator name="Thruster...">` blocks | `origin` sets the mount point and direction; `thrust_coeff`/`max_rpm` set the strength. `mvp_control` reads thruster positions from TF, so if you move a thruster, also move its `*_thruster_link` in `race_auv_description/urdf/base.urdf`. Otherwise the controller will allocate thrust wrongly. The thrust curves (`polynomials`) and limits for the sim are in `race_auv_config/mvp_control_config/config_sim.yaml`. |
| Thruster command order | `race_auv_bringup/config/sim_params.yaml`, `thruster_driver_node.thruster_sub_topics` | Must list the thrusters in the same order as the `<actuator>` blocks in `race_auv.scn`. |
| Sensors (IMU, pressure, DVL, odometry, modem, cameras) | `race_auv.scn`, `<sensor>` blocks | Rate, noise, mounting `origin` and ROS topic. If you move a sensor, update its frame in `race_auv_description/urdf/base.urdf` to match. IMU orientation offsets are in `sim_params.yaml` (`imu_driver_node`). |
| Camera image size, FOV | `race_auv.scn`, `cam_front` / `cam_down` | Stonefish also publishes a matching `CameraInfo`, so the sim AprilTag pipeline picks up the change automatically. |
| Docking-station AprilTags (IDs, size, placement) | `world_of_stonefish/vehicles/race_station.scn` (`apriltag_*` links) and `race_station/race_station_description/urdf/base.urdf` | The detector finds the tag in the `.scn` image, and the fuser solves the station pose from the URDF. Keep the two files in sync, along with the `tags:` sizes in `race_auv_bringup/config/simulation/apriltag.yaml`. Tag textures are defined in `world_of_stonefish/metadata/looks.scn`. |
| Material densities, colours/textures | `world_of_stonefish/metadata/materials.scn`, `metadata/looks.scn` | Shared by every scenario. |

### Mass and buoyancy
Stonefish computes the vehicle's mass and buoyancy from its parts in `race_auv.scn`:
- **`Hull`**: `buoyant="false"` with an explicit `<mass>` and `<cg>`. It carries most of the dry mass and adds no buoyancy.
- **`left_leg`, `right_leg`, `couple_link`**: their mass is mesh volume × material density, and they are buoyant.
- **`BigFoam`, `SmallFoam`**: `LightFoam` boxes (192 kg/m³) that provide almost all of the buoyancy. Their position sets the center of buoyancy (CB).

The current values match the measured real AUV: **33.5 kg total** with the **CG at x = −0.65 m** from the nose tip. The CB is also at −0.65 m, and net buoyancy is about **+0.3 kg** at a water density of 1023 kg/m³. With these values the vehicle floats level at the surface.

To retune the model:
- **Total mass:** change the `Hull` `<mass>`.
- **CG:** change the `Hull` `<cg>`. The total CG is the mass-weighted average of the hull CG and the other parts, so the hull CG sits slightly aft of the target.
- **Net buoyancy:** change the foam box `dimensions`.
- **Trim:** keep the foam `origin` x at the same position as the CG. If the CB and CG are offset along x, the vehicle pitches, because the CB is only about 5 cm above the CG.

To check a change, run the sim and watch depth and pitch on `/race_auv/stonefish/odometry`.

## Citation

The MVP paper:

```
@inproceedings{
    MVP_PAPER,
    title = {Working toward the development of a generic marine vehicle framework: ROS-MVP},
    author={Gezer, Emir Cem and Zhou, Mingxi and Zhao, LIN and McConnell, William},
    booktitle={OCEANS 2022: Hampton Roads},
    year={2022},
    organization={IEEE}
}
```
