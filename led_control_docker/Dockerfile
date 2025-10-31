# Use roslaunch to auto-start roscore + node
FROM yellowandorange/yoloxtb3:base3
SHELL ["/bin/bash", "-c"]
# ROS
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-rospy \
    ros-noetic-std-msgs \
    python3-catkin-tools \
  && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir rpi.gpio

# Launch Files
WORKDIR /root/ws/src
RUN mkdir -p led_control_pkg
COPY ./led_control_pkg /root/ws/src/led_control_pkg

RUN find /root/ws/src/led_control_pkg -type f -name "*.py" -exec chmod +x {} \; || true

# Catkin Make
WORKDIR /root/ws
RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && catkin_make"
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ws/devel/setup.bash'  >> /root/.bashrc

CMD ["bash", "-lc", "source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && roslaunch led_control_pkg led_control.launch"]
