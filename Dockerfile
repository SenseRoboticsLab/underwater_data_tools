FROM ros:noetic

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Arguments for user ID and group ID
ARG USER_ID=1000
ARG GROUP_ID=1000

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --upgrade pip && \
    pip3 install \
    numpy \
    opencv-python

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Clone the waterlinked_a50_ros_driver package
RUN cd /catkin_ws/src && \
    git clone https://github.com/SenseRoboticsLab/dvl_a50_ros_driver.git

# Build the catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /catkin_ws && \
    catkin_make"

# Create user with matching host user ID
RUN groupadd -g ${GROUP_ID} rosuser || true && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash rosuser && \
    echo "rosuser ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Give rosuser ownership of catkin workspace
RUN chown -R rosuser:rosuser /catkin_ws

# Set up entrypoint to source ROS environment for rosuser
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/rosuser/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /home/rosuser/.bashrc

# Create working directory for scripts
RUN mkdir -p /workspace && chown rosuser:rosuser /workspace
WORKDIR /workspace

# Switch to rosuser
USER rosuser

# Copy the script files
COPY --chown=rosuser:rosuser bag2raw.py /workspace/
COPY --chown=rosuser:rosuser filterbag.py /workspace/

# Set up the entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && exec \"$@\"", "--"]
CMD ["/bin/bash"]