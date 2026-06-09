
# Sawyer + 3D Systems Touch X Teleop

ROS Noetic workspace for teleoperating a Rethink Robotics **Sawyer** arm with a
**3D Systems Touch X** haptic device, running fully inside Docker.

Pipeline: `Touch X (6-DoF pose)` → `RelaxedIK` → `intera_interface` → `Sawyer`.

---
<p align="centre">
  <img src="touch.jpeg" alt="Alt Text" style="width:50%; height:auto;">
</p> 

press and hold **dark grey** while teleoperating. press **lighter grey** to open/close the gripper.

## RUN the code

#### step 1:
```bash
export REPO_PATH=$HOME/rpl_sawyer
mkdir -p $REPO_PATH
cd $REPO_PATH
git clone https://github.com/dqhuangnus/Sawyer_teleop_with_touchx-3D-system.git
cd Sawyer_teleop_with_touchx-3D-system
```

Install the Touch X driver on the host (one-time — registers udev rules so the container can see the device over USB):
#### step 2:
```bash
cd external/TouchDriver
./install_haptic_driver
cd ../..
```
#### step 3: 
```bash
docker build -t sawyer_haptic .
```

if step 3, gives permission denied error when running docker commands, add your user to the docker group:
```bash
sudo usermod -aG docker $USER
newgrp docker
```

once fixed, repeat the **step 3** again.

#### step 4:

```bash
xhost +local:root

docker run -it --privileged --net=host \
  --name sawyer_haptic \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/external/OpenHaptics:/opt/OpenHaptics:ro \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/external/TouchDriver:/root/TouchDriver_2024_09_19:ro \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/external/TouchLibs:/usr/lib/TouchLibs:ro \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/collected_data:/root/collected_data \
  -v /dev/bus/usb:/dev/bus/usb \
  -w /root/sawyer_haptic_workspace \
  sawyer_haptic:latest
```
#### step 5:
Edit `intera.sh` — set `robot_hostname` to the robot IP and `your_ip` to your computer's IP. Then source it:

```bash
source devel/setup.bash
nano intera.sh
source intera.sh
```

Test ROS comms:

```bash
rostopic list
```

Terminal 1:

```bash
roslaunch omni_common omni_state.launch
```
#### step 6:
Terminal 2:

```bash
sudo docker ps
sudo docker exec -it <container_name_or_id> bash
source devel/setup.bash
source intera.sh
roslaunch touchx_sawyer_teleop test_touchx_viz.launch
```

### NOTE: you might need to change some directory inside the urdf and launch file.

---

### how it works:

once both terminals are running, you will notice that the touch has two button:
- press dark grey to teleoperate
- press light grey for open and closing the gripper.

---

## Data collection (Basler + uSkin → tactile-ACT episodes)

Recording is **integrated into the teleop node**: one `xela_server` + one launch
captures synchronised sensor + robot state into `episode_*.hdf5` files in the
**tactile_act_real** format.

**Sensors & how they're wired in:**

| Sensor | Package (in image) | Interface | HDF5 keys |
|--------|--------------------|-----------|-----------|
| Basler ×3 (GigE) | `pypylon` | by IP over `--net=host` | `image_left/right/top` (T,300,480,3) |
| uSkin (2 fingers) | `websocket-client` → `xela_server` | SocketCAN → `ws://localhost:5000` | `tactile_1/2` (T,5,24,3) |
| Intel RealSense *(off by default)* | `pyrealsense2` | USB | `image_realsense` (T,480,640,3) + `depth_realsense` (T,480,640) |

Plus `action_pos` (T,3), `action_quat` (T,4), `gripper` (T,1), `joint_state` (T,7),
`timestamp` (T). RealSense is disabled by default — set `record_realsense:=true` in
[touchx_teleop.launch](src/touchx_sawyer_teleop/launch/touchx_teleop.launch) to add it.

### Full workflow

**A. Host — one-time prep**
```bash
export REPO_PATH=$HOME/rpl_sawyer
cd $REPO_PATH/Sawyer_teleop_with_touchx-3D-system

# proprietary deps under external/ :
#   external/OpenHaptics, external/TouchDriver, external/TouchLibs  (3D Systems)
#   external/Xela/xela_server + external/Xela/xServ.ini            (XELA uSkin server)

cd external/TouchDriver && ./install_haptic_driver && cd ../..   # Touch X udev rules
sudo ip link set can0 up type can bitrate 1000000                # uSkin CAN bus up
xhost +local:root                                                 # allow RViz GUI
```

**B. Build**
```bash
docker build -t sawyer_haptic .
```

**C. Run** (add a `collected_data` volume so episodes land on the host)
```bash
docker run -it --privileged --net=host \
  --name sawyer_haptic \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/external/OpenHaptics:/opt/OpenHaptics:ro \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/external/TouchDriver:/root/TouchDriver_2024_09_19:ro \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/external/TouchLibs:/usr/lib/TouchLibs:ro \
  -v $REPO_PATH/Sawyer_teleop_with_touchx-3D-system/collected_data:/root/collected_data \
  -v /dev/bus/usb:/dev/bus/usb \
  -w /root/sawyer_haptic_workspace \
  sawyer_haptic:latest
```

**D. Container — network + enable robot**
```bash
source devel/setup.bash
nano intera.sh            # set robot_hostname (robot IP) and your_ip (this PC)
source intera.sh
rostopic list             # verify comms
rosrun intera_interface enable_robot.py -e
```

**E. Container — tactile server + teleop/record (two terminals)**

The two terminals must be inside the **same** container. Open the second one with
`docker exec` (find the name/ID via `docker ps`):
```bash
# terminal 1 (the `docker run` shell above) — uSkin websocket server, leave running:
xela_server -f /etc/xela/xServ.ini --port 5000 --ip 0.0.0.0

# terminal 2 — attach to the same container:
docker exec -it sawyer_haptic bash
source devel/setup.bash && source intera.sh
roslaunch touchx_sawyer_teleop touchx_teleop.launch
```

**F. Collect** (in the teleop terminal)

| Action | Control |
|--------|---------|
| Teleoperate | hold Touch X **dark grey** button |
| Open/close gripper | Touch X **light grey** button |
| Return to home | `h` |
| **Start recording an episode** | `r` |
| **Finish + save** | `f` |
| Discard current episode | `d` |

Files are written to `collected_data/episode_<timestamp>_epNNN.hdf5` on the host.

### Troubleshooting collection
- **Tactile all zeros / `tactile_1/2` empty** → `can0` not up on host, or `xela_server` not started first.
- **No camera images** → Basler IPs / subnet wrong (cameras must be reachable; `--net=host` already shares the network).
- **RealSense needed** → set `record_realsense:=true` in the launch (USB passthrough already provided by `--privileged` + `/dev/bus/usb`).

---

## Troubleshooting

### `OSError: ... librelaxed_ik_lib.so: cannot open shared object file`
`relaxed_ik_core` is a Rust library whose compiled `.so` is **not** committed to git
(`target/` is a build artifact). The Docker image builds it for you. If you see this error:

- **Rebuild the image** (`docker build -t sawyer_haptic .`) if you built it from an older
  version of this repo — the fix lives in the image, not in a mounted volume.
- **Running outside Docker?** Build the library by hand:
  ```bash
  cd src/relaxed_ik_core
  cargo build --release
  # the wrapper loads target/debug/, so point it at the release build:
  mkdir -p target/debug
  ln -sf ../release/librelaxed_ik_lib.so target/debug/librelaxed_ik_lib.so
  ```

### `libGL error: failed to load driver: nouveau`
Harmless — RViz can't get hardware OpenGL inside the container and falls back to software
rendering. To silence it: `export LIBGL_ALWAYS_SOFTWARE=1`.
