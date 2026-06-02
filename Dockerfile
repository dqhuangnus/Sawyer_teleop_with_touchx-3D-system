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
    && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install "numpy==1.17.4" "pyRobotiqGripper==1.0.1"

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
