# Partimos de la imagen oficial de ROS2 Humble
FROM osrf/ros:humble-desktop-full

# Instalar paquetes del TurtleBot4 y Nav2
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot4-simulator \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-viz \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Configurar el .bashrc
RUN echo "" >> ~/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "export ROS_DOMAIN_ID=0 # TurtleBot4" >> ~/.bashrc \
    && echo "export ROS_LOCALHOST_ONLY=1 # multicast OFF" >> ~/.bashrc \
    && echo "" >> ~/.bashrc \
    && echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc \
    && echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# Directorio de trabajo
WORKDIR /root
