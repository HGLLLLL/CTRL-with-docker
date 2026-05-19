FROM ros:noetic

# 指定終端機顏色與設定 apt-get 為非互動模式
ENV TERM=xterm-256color
ENV DEBIAN_FRONTEND=noninteractive

# 0. 解決 ROS GPG Key 過期問題 (先移除舊源 -> 更新 Ubuntu -> 裝 curl -> 抓新 Key -> 重新加入 ROS 源)
RUN rm -f /etc/apt/sources.list.d/ros*.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends curl gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros/ubuntu focal main" | tee /etc/apt/sources.list.d/ros1.list > /dev/null

# 1. 集中安裝系統依賴、ROS 套件，並在同一層清理 apt 快取以大幅縮減 Image 大小
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      git \
      zsh \
      tmux \
      htop \
      neovim \
      neofetch \
      python3-pip \
      libeigen3-dev \
      ros-noetic-tf2-* \
      ros-noetic-controller-manager \
      ros-noetic-transmission-interface \
      ros-noetic-image-geometry \
      libcv-bridge-dev \
      python3-cv-bridge \
      ros-noetic-cv-bridge \
      ros-noetic-image-transport \
      ros-noetic-xacro \
      ros-noetic-robot-state-publisher \
      ros-noetic-rqt-tf-tree \
      ros-noetic-rosserial \
      ros-noetic-rosserial-arduino \
      ros-noetic-rosserial-python \
      ros-noetic-rplidar-ros \
      python3-scipy \
      python3-matplotlib \
      && rm -rf /var/lib/apt/lists/*

# 2. 集中安裝 Python 相關套件 (加上 --no-cache-dir 避免快取佔用空間)
RUN pip3 install --no-cache-dir \
    pyserial \
    qrcode_terminal \
    tornado \
    filterpy
    # torch \
    # torchvision \
    # ultralytics

# 3. 安裝與設定 Zsh, Oh My Zsh 及相關外掛
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting && \
    chsh -s $(which zsh)

# 複製使用者的設定檔
COPY dotfiles/.p10k.zsh /root/.p10k.zsh
COPY dotfiles/.zshrc /root/.zshrc
COPY cachefile/gitstatus /root/.cache/gitstatus

WORKDIR /root/catkin_ws

# 複製 rules 與 entrypoint 並設定執行權限
COPY scripts/*.rules /root/scripts/
COPY scripts/entrypoint.sh /root/scripts/entrypoint.sh
RUN chmod +x /root/scripts/entrypoint.sh

# Entry point
ENTRYPOINT ["/root/scripts/entrypoint.sh"]
CMD ["zsh"]