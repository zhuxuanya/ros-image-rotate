# ros-image-rotate

This ROS node listens to image messages, rotates them according to the specified angle and scale, and publishes to a new topic. It helps in camera image rotation under real-time conditions.

## Installation

Navigate to the `src` directory of the catkin workspace (e.g., `~/catkin_ws`):

```bash
cd ~/catkin_ws/src
```

Clone the repository into `src` directory and rename it to `image_rotate`:

```bash
git clone https://github.com/zhuxuanya/ros-image-rotate.git image_rotate
```

Return to catkin workspace root and rebuild:

```bash
cd ..
catkin_make
```

Source the environment to update the workspace:

```bash
source devel/setup.bash
```

## Usage

List all connected video devices to determine the correct device number (e.g., `/dev/video0`):

```bash
# sudo apt install v4l-utils
v4l2-ctl --list-devices
```

Modify the corresponding device parameter in `usb_cam` node and run it with the `image_rotate` node:

```bash
roslaunch image_rotate image_node.launch
```
