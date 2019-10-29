sudo apt-get update
yes | sudo apt-get install cmake g++
echo "git, cmake and g++ install"

sudo swapoff -a
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo bash -s 'echo "/swapfile swap swap defaults 0 0" >> /etc/fstab'

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt-get update
echo "Update done"
yes | sudo apt-get install curl gnupg2 lsb-release
echo "Curl, gnupg2, lsb-release installed"
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt-get update
echo "Re-update"
yes | sudo apt-get dist-upgrade
echo "Dist-upgrade complete"
yes | sudo apt-get install ros-dashing-ros-base
echo "Ros2 installed"

source /opt/ros/dashing/setup.bash
echo "source /opt/ros/dashing/setup.bash" >> ~/.bashrc

yes | sudo apt-get install python3-argcomplete python3-colcon-common-extensions libboost-system-dev
echo "apt-get complete"
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
git clone -b ros2 https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b ros2 https://github.com/ROBOTIS-GIT/DynamixelSDK.git

cd ~/turtlebot3_ws/src/turtlebot3
rm -r turtlebot3_cartographer turtlebot3_navigation2
cd ~/turtlebot3_ws/
source /opt/ros/dashing/setup.bash
colcon build --symlink-install --parallel-workers 1
echo "turtlebot3_ws built"

echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42 #TURTLEBOT3' >> ~/.bashrc
source ~/.bashrc

cd ~/turtlebot3_ws/src/turtlebot3/turtlebot3_bringup
sudo cp ./99-turtlebot3-cdc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

sudo dpkg --add-architecture armhf
sudo apt-get update
echo "updated again"
yes | sudo apt-get install libc6:armhf
echo "libc6 installed"

cd && rm -rf opencr_update.tar.bz2
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xjf ./opencr_update.tar.bz2

export OPENCR_PORT=/dev/ttyACM0
export OPENCR_MODEL=burger
cd ~/opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr
