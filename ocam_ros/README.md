# ocam_ros

A ROS wrapper for the OCAM Global Shutter Camera

## Functionality
- Display Image and publish to ROS network
- Use the `~show_image` parameter to adjust brightness and exposure on the fly

## Supported Image Formats
* USB 3.0
  * 8-bit Greyscale 1280 x 720 @ 60 fps
  * 8-bit Greyscale 1280 x 960 @ 45 fps
  * 8-bit Greyscale 320 x 240 @ 160 fps
  * 8-bit Greyscale 640 x 480 @ 80 fps
* USB 2.0
  * 8-bit Greyscale 1280 x 720 @ 30 fps
  * 8-bit Greyscale 1280 x 960 @ 22.5 fps
  * 8-bit Greyscale 320 x 240 @ 160 fps
  * 8-bit Greyscale 640 x 480 @ 80 fps

## Installation
Install dependencies:
``` bash
sudo apt install libudev-dev libv4l-dev
```
Compile the ROS package:

``` bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone https://superjax/ocam_ros
cd ..
catkin_make
```

## Running the Node

```bash
rosrun ocam_ros ocam_ros_node
```

For changing parameter values and topic remapping from the command line using `rosrun` refer to the [Remapping Arguments](http://wiki.ros.org/Remapping%20Arguments) page.

For setting parameters and topic remappings from a launch file, refer to the [Roslaunch for Larger Projects](http://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) page.

## Parameters
* `~device_path` (string, default: "/dev/video0")
    - Serial port to connect to
* `~frame_id` (string, default: "camera")
   - Frame id of camera measurements
* `~camera_info_url` (string, default: "")
   - Path to camera intrinsic parameters file
* `~width` (int, default: 640)
   - Image width in pixels
* `~height` (int, default: 480)
   - Image height in pixels
* `~fps` (int, default: 80)
   - Camera frame rate
* `~image_topic` (string, default: "image_raw")
   - Color image topic name
* `~mono_image_topic` (string, default: "image_mono")
   - Greyscale image topic name
* `~show_image` (bool, default: false)
   - Shows image and enables keyboard shortcuts to adjust image
* `~rescale_camera_info` (bool, default: false)
   - Whether or not rescale the camera intrinsics
* `~brightness` (int, default: 64, min/max: 1/127)
   - Adjusts image brightness
* `~exposure` (int, default: 39, min/max: 1/625)
   - Adjusts camera shutter speed
* `~auto_exposure` (bool, default: true)
   - Whether or not to automatically adjust exposure

## Topics
- `camera/image_raw`(sensor_msgs/Image)
    - Color image measurements from oCam
- `camera/image_mono`(sensor_msgs/Image)
    - Greyscale image measurements from oCam

## ToDo
- Finish documentation
- Adjusting rate of streaming and configuration on the fly with rqt_reconfigure
