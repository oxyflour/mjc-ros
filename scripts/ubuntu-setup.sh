# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#setup-sources
#

if false; then
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
fi

if false; then
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
fi

if false; then
    export HTTPS_PROXY=http://proxy.yff.me:8124/
    sudo apt update && sudo apt install curl -y
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
    sudo dpkg -i /tmp/ros2-apt-source.deb
fi

if true; then
    sudo apt update && \
    sudo apt install ros-humble-desktop
fi

colcon
if [ $? -ne 0 ]; then
    sudo apt install python3-colcon-common-extensions
fi

ros2 bag info --storage mcap
if [ $? -ne 0 ]; then
    sudo apt install ros-humble-rosbag2-storage-mcap
fi

# source /opt/ros/humble/setup.bash
