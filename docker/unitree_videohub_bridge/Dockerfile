# Image with unitree_videohub_bridge and everything it needs to run: the head camera of a Unitree robot (G1, H1),
# read through Unitree's videohub service, published on ROS 2. Build it from the root of the repository (the build
# context is the repository, which is copied into the image):
#
#   docker build -t unitree_videohub_bridge -f docker/unitree_videohub_bridge/Dockerfile .
#
# For the onboard computer of the robot (arm64), from a desktop computer:
#
#   docker build --platform linux/arm64 -t unitree_videohub_bridge -f docker/unitree_videohub_bridge/Dockerfile .
#
# Run it on the onboard computer, on the host network (no access to the video devices is needed):
#
#   docker run -it --rm --net=host --ipc=host unitree_videohub_bridge \
#       bash -c "ros2 launch unitree_videohub_bridge unitree_videohub_bridge_launch.py robot:=g1"
#
# The base image is the one of the SAS drivers of the robots: ROS 2 Jazzy (desktop), OpenCV, image_transport, Eigen,
# DQ Robotics, and sas_cpp are already in it, and its layers are shared with those images. It sources ROS in /etc/bash_env
# (BASH_ENV), which every bash shell, interactive or not, reads.
FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

########################################################################################################
# The image_transport plugins of the published images: "ffmpeg" (H.264; it brings the ffmpeg libraries and the libx264
# encoder) and "compressed" (JPEG). Without them, only the "raw" transport is available. OpenCV, Eigen (unitree_drivers),
# DQ Robotics (built by unitree_drivers, although its videohub library does not use it), and sas_cpp
# (libmarinholab-sas-core: marinholab::sas::core, needed by unitree_drivers) are in the base image, and listed here so
# that a change of the base image does not remove them silently.
RUN apt-get update && \
    apt-get install -y \
        git \
        libopencv-dev \
        libeigen3-dev \
        libdqrobotics \
        libmarinholab-sas-core \
        ros-jazzy-image-transport \
        ros-jazzy-ffmpeg-image-transport \
        ros-jazzy-compressed-image-transport && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/git

########################################################################################################
# unitree_sdk2, in /opt/unitree_robotics. It bundles CycloneDDS (libddscxx.so.0 etc.) in /opt/unitree_robotics/lib,
# which the dynamic linker does not search by default: registered with ldconfig, so that the executables find it at
# run time.
RUN cd ~/git/ && \
    git clone --recursive https://github.com/unitreerobotics/unitree_sdk2.git && \
    cmake -S ~/git/unitree_sdk2 -B ~/git/unitree_sdk2/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics && \
    cmake --build ~/git/unitree_sdk2/build -j$(nproc) && \
    cmake --install ~/git/unitree_sdk2/build && \
    echo "/opt/unitree_robotics/lib" > /etc/ld.so.conf.d/unitree_robotics.conf && \
    ldconfig

########################################################################################################
# unitree_drivers (DriverUnitreeVideohub). ldconfig: the libraries in /usr/local/lib are not found at run time until
# the cache of the dynamic linker is updated.
RUN cd ~/git/ && \
    git clone https://github.com/Adorno-Lab/unitree_drivers.git && \
    cmake -S ~/git/unitree_drivers -B ~/git/unitree_drivers/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_PREFIX_PATH=/opt/unitree_robotics && \
    cmake --build ~/git/unitree_drivers/build -j$(nproc) && \
    cmake --install ~/git/unitree_drivers/build && \
    ldconfig

########################################################################################################
# unitree_videohub_bridge, from the build context (this repository), in its own ROS 2 workspace. Last, so that a
# change of the package only rebuilds this layer.
COPY . /ros2_ws/src/unitree_videohub_bridge
RUN cd /ros2_ws && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    echo "source /ros2_ws/install/setup.bash" >> /etc/bash_env

WORKDIR /ros2_ws
CMD ["/bin/bash"]
