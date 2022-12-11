FROM moveit/moveit2:rolling-release

SHELL ["/bin/bash", "-c"]

ENV HOME=/home/roboprinter
ENV ROS2_WORKSPACE=roboprinter_ws

WORKDIR $HOME/$ROS2_WORKSPACE
COPY . $HOME/$ROS2_WORKSPACE/src

#RUN source /opt/ros/rolling/setup.bash 
#RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN source /opt/ros/rolling/setup.bash  && colcon build --packages-select roboprinter

#ENTRYPOINT [ "ru"]
#RUN echo "source $HOME/$ROS2_WORKSPACE/install/setup.bash" >> /root/.bashrc

CMD [ "/bin/bash", "-c" ]
