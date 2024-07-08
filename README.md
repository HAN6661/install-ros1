# install-ros1
ROS (Robot Operating System) is a framework for developing robot software, offering tools and libraries to simplify creating complex behaviors. ROS Noetic Ninjemys is the latest LTS release for Ubuntu 20.04, providing long-term stability and enhanced performance, supporting modular and collaborative development.


## Key Features of ROS Noetic:

  1- Modularity: ROS is designed to be modular, allowing you to write small, reusable pieces of code that can be integrated together.
  2-Tools and Libraries: ROS provides a wide range of tools and libraries for tasks such as hardware abstraction, device control, and message-passing.
  3-Community and Ecosystem: ROS has a large, active community and a rich ecosystem of packages and tutorials.

## Installation Guide for ROS Noetic
###  1- Setup your sources.list: First, you need to configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse."
   
   ```
      sudo apt update
      sudo apt install software-properties-common
      sudo add-apt-repository universe
      sudo add-apt-repository multiverse
      sudo add-apt-repository restricted
   ```

Then, add the ROS repository to your sources list.

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

 ### 2- Set up your keys: Add the ROS GPG key to your system to verify the integrity of the packages. 
 
   ```
 sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

   ```

 ### 3- Installation:Update your package list and install ROS Noetic.

  ```
sudo apt update
sudo apt install ros-noetic-desktop-full

```
The desktop-full installation includes a wide array of tools and libraries, including visualization and simulation tools. Alternatively, for a minimal installation, you can use:

```
sudo apt install ros-noetic-ros-base

```
### 4- Initialize rosdep:rosdep is a command-line tool for installing system dependencies for the source you want to compile.

```
sudo rosdep init
rosdep update

```

### 5- Environment Setup: Add the ROS environment setup script to your .bashrc file so that the ROS environment variables are automatically added to your shell session each time you open a terminal.

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

```

### 6- Dependencies for building packages: Install additional tools for building and managing ROS packages.

```
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

```

### 7- Create and build a Catkin workspace: Catkin is the official build system of ROS. You need to create a Catkin workspace to build and store your packages.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

```
Source your new workspace:

```
source devel/setup.bash

```

## Verification:
To verify the installation, you can run a simple ROS command:

```
roscoer
```
If roscore starts without errors, your ROS Noetic installation is successfully set up.
