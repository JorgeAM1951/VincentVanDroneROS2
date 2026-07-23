FROM osrf/ros:jazzy-desktop-full

SHELL ["/bin/bash", "-c"]

# Configurar Python3 por defecto
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Instalar herramientas del sistema y utilidades de ROS 2[cite: 14]
RUN apt-get update && apt-get upgrade -y \
  && apt-get install -y \
    tmux \
    tmuxinator \
    vim \
    xterm \
    curl \
    wget \
    ssh \
    tree \
    python3-tk \
    nano \
    libtool \
    libtool-bin \
    htop \
    gdb \
    net-tools \
    build-essential \
    cmake \
    lsb-release \
    iputils-ping \
    ninja-build \
    python3-rosdep \
    python3-setuptools \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    ros-dev-tools \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN echo "set -g mouse on" > /root/.tmux.conf

# Instalar dependencias de Python (Librerías del optimizador + utilidades)[cite: 14]
RUN pip3 install colcon-lcov-result cpplint cmakelint PySimpleGUI-4-foss numpy scipy matplotlib clarabel osqp --break-system-packages

# Instalar MAVROS y descargar los datasets geográficos necesarios
RUN apt-get update && apt-get install -y \
    ros-jazzy-mavros \
    ros-jazzy-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN wget -O /tmp/install_geographiclib_datasets.sh https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x /tmp/install_geographiclib_datasets.sh \
    && /tmp/install_geographiclib_datasets.sh

# Crear y configurar el Workspace de ROS 2
RUN mkdir -p /root/drones_unizar_ws/src
WORKDIR /root/drones_unizar_ws

# Configurar el bashrc para cargar ROS y el workspace automáticamente[cite: 14]
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/drones_unizar_ws/install/setup.bash" >> /root/.bashrc

# (Opcional) Inicializar rosdep si fuera necesario en el futuro
# RUN rosdep init || true
# RUN rosdep update