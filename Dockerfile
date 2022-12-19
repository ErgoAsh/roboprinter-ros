FROM moveit/moveit2:rolling-release

ENV HOME=/home/roboprinter
ENV ROS2_WORKSPACE=roboprinter_ws
ENV ROS_DISTRO=rolling

WORKDIR $HOME/$ROS2_WORKSPACE
COPY roboprinter_description src/roboprinter_description/
COPY roboprinter_moveit_config src/roboprinter_moveit_config/
COPY roboprinter src/roboprinter/

RUN . /opt/ros/rolling/setup.sh && \
    colcon build --packages-select \
  	roboprinter_description \ 
	roboprinter_moveit_config \
	roboprinter 

RUN echo "source /opt/ros/$ROS_DISTRO/install/setup.bash" >> /root/.bashrc
RUN echo "source $HOME/$ROS2_WORKSPACE/install/local_setup.bash" >> /root/.bashrc

COPY ./roboprinter_entrypoint.sh /roboprinter_entrypoint.sh
ENTRYPOINT [ "/roboprinter_entrypoint.sh" ]
CMD [ "/bin/bash", "-c" ]
#CMD [ "launch" ]
