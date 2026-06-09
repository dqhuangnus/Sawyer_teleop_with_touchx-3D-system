FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# -----------------------------
# 1. Install system + ROS deps
# -----------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    python3-rosdep \
    python3-wstool \
    python3-vcstools \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-catkin \
    ros-noetic-xacro \
    ros-noetic-rviz \
    ros-noetic-tf2-ros \
    ros-noetic-cv-bridge \
    ros-noetic-control-msgs \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-trajectory-msgs \
    ros-noetic-rospy-message-converter \
    ros-noetic-moveit \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-msgs \
    ros-noetic-joint-state-publisher \
    libusb-1.0-0-dev \
    libncurses5-dev \
    libncurses5 \
    freeglut3-dev \
    python3-opencv \
    libfuse2 \
    && rm -rf /var/lib/apt/lists/*


# Python deps. numpy is bumped to 1.24.4 (the old 1.17.4 pin is incompatible
# with h5py); this set is verified to run teleop + RelaxedIK + gripper + capture.
#   pypylon          -> Basler GigE cameras (installed from Basler's cp38 wheel —
#                       the 2.0.0rc1 rc is not on PyPI; bundles the pylon runtime)
#   websocket-client -> uSkin tactile (connects to xela_server @ ws://localhost:5000)
#   h5py             -> episode recording   (cv2 comes from apt python3-opencv)
# RealSense (pyrealsense2) is intentionally NOT installed yet — capture is off by
# default; `pip install pyrealsense2` when you enable record_realsense.
# focal's stock pip (20.0.2) doesn't understand the manylinux_2_28 tag on the
# pypylon wheel — upgrade pip ONLY. (Do not upgrade setuptools: catkin's
# interrogate_setup_dot_py.py needs the apt setuptools, which is pinned to the
# old importlib_metadata; a newer setuptools breaks catkin_make.)
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install \
        "numpy==1.24.4" \
        "pyRobotiqGripper==1.0.1" \
        "https://github.com/basler/pypylon/releases/download/2.0.0rc1/pypylon-2.0.0rc1-cp38-cp38-manylinux_2_27_x86_64.manylinux_2_28_x86_64.whl" \
        "websocket-client==1.8.0" \
        "h5py==3.11.0" \
        "pyyaml"

# Rust toolchain for relaxed_ik_core (pure-Rust crate, not built by catkin_make)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
        | sh -s -- -y --default-toolchain stable --profile minimal
ENV PATH="/root/.cargo/bin:${PATH}"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# -----------------------------
# 2. Catkin workspace
# -----------------------------
WORKDIR /root/sawyer_haptic_workspace

# Copy the full Sawyer + haptic source tree (vendored Sawyer SDK, Intera,
# MoveIt, sns_ik, phantom_omni, Robotiq, relaxed_ik_core, and the custom
# touchx_sawyer_teleop package).
COPY src/ ./src/

# -----------------------------
# 3. 3D Systems Touch X / OpenHaptics (proprietary, supplied by user)
# -----------------------------
# These directories are NOT redistributed via git. Before building the image,
# download the OpenHaptics SDK and Touch Driver from the 3D Systems support
# site and place them under ./external/ (see README.md).
COPY external/OpenHaptics/opt/OpenHaptics/  /opt/OpenHaptics/
COPY external/OpenHaptics/usr/include/      /usr/include/
COPY external/TouchDriver/                  /root/TouchDriver_2024_09_19/
COPY external/TouchLibs/                    /usr/lib/TouchLibs/

RUN echo "/usr/lib/TouchLibs" >> /etc/ld.so.conf.d/touchlibs.conf && \
    ln -sf /usr/lib/TouchLibs/libHD.so.3.4.0          /usr/lib/libHD.so && \
    ln -sf /usr/lib/TouchLibs/libHD.so.3.4.0          /usr/lib/libHD.so.3 && \
    ln -sf /usr/lib/TouchLibs/libHL.so.3.4.0          /usr/lib/libHL.so && \
    ln -sf /usr/lib/TouchLibs/libHL.so.3.4.0          /usr/lib/libHL.so.3 && \
    ln -sf /usr/lib/TouchLibs/libPhantomIOLib42.so    /usr/lib/libPhantomIOLib42.so && \
    ln -sf /usr/lib/TouchLibs/libPhantomManagerLite.so /usr/lib/libPhantomManagerLite.so && \
    cp    /usr/lib/TouchLibs/libHDU.a                 /usr/lib/libHDU.a && \
    ldconfig

ENV OH_SDK_BASE=/opt/OpenHaptics
ENV LD_LIBRARY_PATH=/usr/lib/TouchLibs:${LD_LIBRARY_PATH:-}

# -----------------------------
# 3b. XELA / uSkin tactile server (proprietary, supplied by user)
# -----------------------------
# The uSkin websocket server (ws://localhost:5000) is vendor software, not
# redistributed via git. Before building, place the XELA server binary and its
# config under ./external/Xela/ (see README.md):
#     external/Xela/xela_server   -> /usr/local/bin/xela_server  (~57 MB binary)
#     external/Xela/xServ.ini      -> /etc/xela/xServ.ini         (sensor config)
# Start it at runtime with:  xela_server -f /etc/xela/xServ.ini --port 5000 --ip 0.0.0.0
# Build still succeeds without these — only tactile capture is unavailable.
COPY external/Xela/ /tmp/xela/
RUN if [ -f /tmp/xela/xela_server ]; then \
        install -m 0755 /tmp/xela/xela_server /usr/local/bin/xela_server && \
        mkdir -p /etc/xela && \
        ([ -f /tmp/xela/xServ.ini ] && cp /tmp/xela/xServ.ini /etc/xela/xServ.ini || true) && \
        echo "[build] XELA server installed"; \
    else \
        echo "[build] external/Xela/xela_server absent — tactile server not installed"; \
    fi && rm -rf /tmp/xela

# -----------------------------
# 4. rosdep + build workspace
# -----------------------------
RUN rosdep init || true
RUN rosdep update
RUN cd /root/sawyer_haptic_workspace && \
    rosdep install --from-paths src --ignore-src -r -y || true

RUN source /opt/ros/noetic/setup.bash && \
    cd /root/sawyer_haptic_workspace && \
    catkin_make -j$(nproc)

# Build the relaxed_ik_core Rust library (release for speed; the Python wrapper
# loads target/debug/librelaxed_ik_lib.so, so symlink the debug path to it).
RUN cd /root/sawyer_haptic_workspace/src/relaxed_ik_core && \
    cargo build --release && \
    mkdir -p target/debug && \
    ln -sf ../release/librelaxed_ik_lib.so target/debug/librelaxed_ik_lib.so

# -----------------------------
# 5. Intera shell script + bashrc
# -----------------------------
COPY intera.sh ./intera.sh


RUN echo "source /opt/ros/noetic/setup.bash"                       >> /root/.bashrc && \
    echo "source /root/sawyer_haptic_workspace/devel/setup.bash"   >> /root/.bashrc

# -----------------------------
# 6. Utilities
# -----------------------------
RUN apt-get update && apt-get install -y \
    net-tools nano vim iputils-ping \
    && rm -rf /var/lib/apt/lists/*

CMD ["/bin/bash"]
