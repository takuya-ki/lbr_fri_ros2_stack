FROM ros:humble-ros-base-jammy

# change default shell to bash
SHELL ["/bin/bash", "-c"]

# upgrade packages and clean up in a single layer
RUN apt-get update && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/*

# create workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws/

COPY . /home/ros2_ws/src/
RUN rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO} -y

# "--symlink-install" allows the code in the locally mounted volume to be adjusted without rebuilding
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

# source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /home/ros2_ws/install/local_setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
