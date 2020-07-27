# **delivery_navigation package**
This repo have two function. The first one is ROS navigation. The second one is web interface for delivery service.
We have a short clip (Click bellow image).

[![](http://img.youtube.com/vi/3L4T78DpFC4/0.jpg)](http://www.youtube.com/watch?v=3L4T78DpFC4 "")

# 1. Features
- This pkg can control `pioneer 3-DX` manually.
- This pkg can make 2D map using `slam_gmapping`
- This pkg can execute `ROS navigation`
- This pkg have `web function` of delivery system

# 2. Requirements
- Hardware
  - Base robot : `pioneer 3-DX`
  - Controller : `DualShock3 (DS3)`

![Pioneer 3-DX](https://robots.ros.org/assets/img/robots/pioneer-3-dx/image.jpg)<img src="https://upload.wikimedia.org/wikipedia/commons/1/1a/PlayStation3-DualShock3.png" width=200px>

- Software
  - Ubuntu 16.04
  - ROS kinetic

# 3. Installation
- `DualShock3 driver` (You can skip here if you don't use DS3)
  ```vb
  $ sudo apt-get install dialog build-essential pyqt4-dev-tools libusb-dev libbluetooth-dev python-dbus -y
  $ wget https://github.com/RetroPie/sixad/archive/master.zip -O sixad-master.zip
  $ unzip sixad-master.zip
  $ cd sixad-master
  $ make
  $ sudo make install
  ```
  Reference (As of 2020.7.23)
  [Link](https://askubuntu.com/questions/913599/how-to-connect-dualshock-3-controller-ps3-sixaxis-gamepad-on-ubuntu-16-04)

- `Related pkg`
  ```vb
  # This is very important, because aria support is end of life.
  $ sudo apt install libaria-dev

  # joy node
  $ sudo apt-get install ros-kinetic-joy -y
  $ sudo apt-get install ros-kinetic-joystick-drivers -y

  # RosAria and delivery_navigation
  $ cd catkin_ws/src
  $ git clone https://github.com/amor-ros-pkg/rosaria.git
  $ git clone https://github.com/kumahika/delivery_navigation.git
  $ cd .. | catkin_make
  ```

# 4. Execution
- Paring with DS3
  ```vb
  # Connect laptop and DS3 by USB cable
  $ sudo sixpair
  # Disconnect the cable and push PS button
  $ sudo sixad -s
  ```

- Execution pioneer 3-DX

  Turn on the power of pioneer 3-DX (like this image), connect pioneer and laptop using USB-RS232C cable.
  
  <img src="https://github.com/kumahika/delivery_navigation/blob/master/img/pioneer_interface.jpg" width=400px>
  
  ```vb
  $ sudo chmod 777 /dev/ttyUSB0
  ```

- Execute any command below depending on your purpose
  ```vb
  # This needs each terminal
  $ cd catkin_ws | source devel/setup.bash

  # Manual control
  $ roslaunch delivery_navigation manual_control.launch

  # Creation a map using slam_gmapping
  $ roslaunch delivery_navigation slam_gmapping_realtime.launch
  $ roslaunch delivery_navigation slam_gmapping_offline1.launch
  $ rosbag record /scan /tf
  $ roslaunch delivery_navigation slam_gmapping_offline2.launch
  $ rosrun map_server map_saver

  # ROS navigtion
  $ roslaunch delivery_navigation navigation.launch

  # web
  $ roslaunch delivery_navigation server.launch
  $ rosrun delivery_navigation delivery_from_web_node

  # delivery system
  $ roslaunch delivery_navigation navigation_web.launch
  ```
