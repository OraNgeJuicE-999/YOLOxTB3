FROM yellowandorange/yoloxtb3:base2

WORKDIR /root/ws
RUN source /opt/ros/noetic/setup.bash && catkin_make

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/ws/devel/setup.bash" >> /root/.bashrc
# Run roscore
ENV TURTLEBOT3_MODEL=burger
# roslaunch will auto-start roscore if none is running
CMD ["bash", "-lc", "source /opt/ros/noetic/setup.bash && source /root/ws/devel/setup.bash && roslaunch turtlebot3_bringup turtlebot3_robot.launch"]