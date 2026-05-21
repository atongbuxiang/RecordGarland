ARG ROS_DISTRO=humble
FROM richardarducam/depthai-ros:${ROS_DISTRO}-latest

ENV DEBIAN_FRONTEND=noninteractive

ENV WS=/OAK_WS
RUN mkdir -p /OAK_WS/src
COPY ./ .$WS/src/ros2_oak_ffc_sync
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y
RUN cd $WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && MAKEFLAGS="-j1 -l1" colcon build

RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc

ENTRYPOINT [ "/OAK_WS/src/ros2_oak_ffc_sync/entrypoint.sh" ]
CMD ["zsh"]