FROM yellowandorange/yoloxtb3:edgebase

# Source ROS
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Run roscore
ENV TURTLEBOT3_MODEL=burger
# roslaunch will auto-start roscore if none is running
CMD ["roslaunch","turtlebot3_bringup","turtlebot3_robot.launch"]