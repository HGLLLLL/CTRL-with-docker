FROM ros:noetic


# Specify terminal color
ENV TERM=xterm-256color
# Set non-interactive frontend for apt-get to avoid getting stuck on prompts
ENV DEBIAN_FRONTEND=noninteractive


# clear and reload apt cache
# RUN apt clean && apt update


# Install nvim
# RUN apt-get update && apt-get install -y neovim
RUN rm -f /etc/apt/sources.list.d/ros*.list


RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      curl \
      gnupg && \
    rm -rf /var/lib/apt/lists/*


RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg


RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros-latest.list > /dev/null


RUN apt-get update && \
    apt-get install -y neovim


# Install zsh
RUN apt-get update && apt-get install -y zsh
RUN chsh -s $(which zsh)


# Install oh-my-zsh
RUN apt-get update && apt-get install -y curl
RUN apt-get update && apt-get install -y git
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"


# Install p10k
RUN git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
RUN sed -i 's/ZSH_THEME="robbyrussel}l"/ZSH_THEME="powerlevel10k\/powerlevel10k"/g' ~/.zshrc
COPY dotfiles/.p10k.zsh /root/.p10k.zsh
COPY dotfiles/.zshrc /root/.zshrc


# copy gitstatus binary
COPY cachefile/gitstatus /root/.cache/gitstatus


# Install multiple zsh plugins
# 1. zsh-autosuggestions
# 2. zsh-syntax-highlighting
RUN git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
RUN git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting
RUN sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting)/g' ~/.zshrc


# Install neofetch
RUN apt-get update && apt-get install -y neofetch


# Install libeigen3-dev
RUN apt-get update && apt-get install -y libeigen3-dev


# Install qt5-default
RUN apt-get update && apt-get install -y qt5-default


# Install ros-noetic-tf2-ros
RUN apt-get update && apt-get install -y ros-noetic-tf2-*


# Install ros-noetic-controller-manager
RUN apt-get update && apt-get install -y ros-noetic-controller-manager


# Install ros-noetic-transmission-interface
RUN apt-get update && apt-get install -y ros-noetic-transmission-interface


# Install ros-noetic-image-geometry
RUN apt-get update && apt-get install -y ros-noetic-image-geometry


# Install libcv-bridge-dev
RUN apt-get update && apt-get install -y libcv-bridge-dev


# Install python3-cv-bridge
RUN apt-get update && apt-get install -y python3-cv-bridge


# Install ros-noetic-cv-bridge
RUN apt-get update && apt-get install -y ros-noetic-cv-bridge


# Install ros-noetic-image-transport
RUN apt-get update && apt-get install -y ros-noetic-image-transport


# Install ros-noetic-laser-geometry
RUN apt-get update && apt-get install -y ros-noetic-laser-geometry


# Install ira_laser_tools for merging scans
# RUN apt-get update && apt-get install -y ros-noetic-ira-laser-tools


# Install ros-noetic-turtlebot3-description
# RUN apt-get update && apt-get install -y ros-noetic-turtlebot3-description


# Install gazebo
# COPY scripts/gazebo.sh /root/scripts/gazebo.sh
# RUN chmod +x /root/scripts/gazebo.sh
# RUN cd /root && /root/scripts/gazebo.sh
# RUN apt-get install ros-noetic-gazebo-ros -y


# Install ros_xacro
RUN apt-get install ros-noetic-xacro -y


# Install firefox for testing GUI
RUN apt-get update && apt-get install -y firefox


# Install rviz
RUN apt-get update && apt-get install -y ros-noetic-rviz


# Allow to run GUI
RUN apt-get update && apt-get install -y libpci-dev
RUN apt-get update && apt-get install -y x11-apps
RUN apt-get update && apt-get install -y qtwayland5
RUN apt-get update && apt-get install -yqq xserver-xorg
RUN apt-get update && apt-get install -y xwayland


# Install realsense2_camera
# RUN apt-get update && apt-get install -y ros-noetic-realsense2-camera


# Download Gmapping
# RUN apt-get install ros-noetic-gmapping -y
# RUN apt-get install ros-noetic-slam-gmapping -y


# Save map and opened by gazebo
# RUN apt-get install ros-noetic-map-server -y


# Download ROS navigation
# RUN apt-get install ros-noetic-navigation -y


# Support robot can be controled by keyboard
# RUN apt-get install ros-noetic-teleop-twist-keyboard -y


# Teleop using rosboard
# RUN apt-get install ros-noetic-rosbridge-server -y


# Install pkgs to generate terminal QR code
RUN apt-get install -y python3-pip
RUN pip install qrcode_terminal tornado


# Install tmux
RUN apt-get install tmux -y


# Yolo visual package
RUN pip install opencv-python-headless
RUN pip install torch torchvision
RUN pip install numpy
RUN pip install ultralytics


# Install rqt_tf_tree
RUN apt-get install ros-noetic-rqt-tf-tree -y



# Install robot state publisher
RUN apt-get install ros-noetic-robot-state-publisher -y


# Install ros-noetic-robot-localization
# RUN apt-get install ros-noetic-robot-localization -y

# Install htop
RUN apt-get install htop -y


# Install pyserial
RUN pip install pyserial


# Install rqt_plot and rosserial_python
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rqt-plot \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python \
    && rm -rf /var/lib/apt/lists/*


# Install pyrealsense2
# RUN pip3 install pyrealsense2


#SSHD SETUP
# 1. Install OpenSSH Server and create directory for sshd to run
RUN apt-get update && apt-get install -y openssh-server && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /var/run/sshd /root/.ssh

RUN sed -i 's/#Port 22/Port 2222/' /etc/ssh/sshd_config

# 2. Copy your public key.
COPY ssh/authorized_keys /root/.ssh/authorized_keys



# 3. Set correct permissions for SSH to work
RUN chmod 700 /root/.ssh && chmod 600 /root/.ssh/authorized_keys


# 4. Secure the SSH configuration by disabling password login
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin prohibit-password/' /etc/ssh/sshd_config \
    && sed -i 's/PasswordAuthentication yes/PasswordAuthentication no/' /etc/ssh/sshd_config
#====================================================================================#


WORKDIR /root/catkin_ws


# Copy .rules files from the host into the Docker image
COPY scripts/*.rules /root/scripts/


# Entry point
COPY scripts/entrypoint.sh /root/scripts/entrypoint.sh
ENTRYPOINT ["/root/scripts/entrypoint.sh"]
CMD ["zsh"]

