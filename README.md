# flir_boson_ethernet

Drivers for running FLIR ethernet interface cameras (BlackFly and Boson)

## Getting Started

### Prerequisites 
- ROS (ros-kinetic-desktop-full) or later
- Spinnaker (download and install from [FLIR website](https://www.flir.com/products/spinnaker-sdk/))
- OpenCV2

### Installation
To install the drivers, first navigate to your catkin workspace. If one does not exist, create one
```bash
mkdir -p ~/catkin_ws/src
```
Clone the repo and build
```bash
cd ~/catkin_ws/src
git clone <repo-url>
cd ~/catkin_ws
catkin_make
source devel/setup.sh
```
## Running Launchers
### Single camera
Connect a BlackFly camera and run
```bash
roslaunch flir_boson_ethernet blackfly.launch
```
If you have a Boson, run
```bash
roslaunch flir_boson_ethernet boson.launch
```
These examples include an image viewer. You should see window pop up with the streaming video displayed.

If you want to demo a Boson and a BlackFly running concurrently, run
```bash
roslaunch flir_boson_ethernet multiple_cameras.launch
```
If you want a demo of stereo vision, connect 2 BlackFlys. Get their IP addresses and modify launch/sync_camera.launch lines 11 and 12. For example, if the left IP address is 169.254.87.157 and the right is 169.254.82.142. Change the lines to:
```xml
<arg name="left_ip" default="169.254.87.157"/>
<arg name="right_ip" default="169.254.82.142"/>
```
Next run the node:
```bash
roslaunch flir_boson_ethernet sync_camera.launch
```
You should see 3 image panes appear: left, right and disparity
### Parameters
* frame_id (string)

    Identifier for publishing frames. Specific to the device
* camera_type (string, accepted values: blackfly, boson)

    Which type of camera to which to connect. Only to be used if there is a single camera of this type connected
* ip_addr (string)

    IP address of the camera to which to connect. Takes precedence over camera_type for connection. In the form e.g. 169.254.87.157
* frame_rate (float)

    Publish rate of frames from the camera. Does not affect the actual capture rate of the cameras

* video_format (string accepted values: MONO_8, MONO_16, COLOR_8)

    Pixel type for the image. Choices are mono 8 and 16 bit and 8 bit color. Note: Boson cannot change to COLOR_8 format
* camera_info_url (string)

    Path to camera info file. This information will be published to the camera_info topic
* width (int, optional)

    Width of the output image
* height (int, optional)

    Height of the output image
* xOffset (int, optional)

    Horizontal offset from left for the viewing window
* yOffset (int, optional)

    Vertical offset from top for the viewing window

### Subscribed topics
In order to control the camera(s) during operation, they subscribe to topics where the user can send messages. Note: when using these with sub-namespaced nodes e.g. flir_boson_ethernet/left, then the topic will be under the sub-namespace
* pixel_format (string, accepted values: MONO_8, MONO_16, COLOR_8)

    Same as video_format parameter
* auto_ffc (bool)

    Sets Boson auto FFC on or off
* ffc (command - no additional parameters)

    Triggers Boson shutter
* set_node (key value pair)

    Sets Spinnaker node map to specified value. Warning: this is unstable. Any change to image size or format may result in a fatal error due to buffer sizes not resizing. Use set_roi or set_center_roi to change image size
* set_roi (region of interest)

    Sets region of interest for the viewing window. Specify x_offset, y_offset, width and height
* set_center_roi (region of interest)

    Sets region of interest to center of the max viewing window. Specify only width and height



### sendCommand script
Included is a script that makes sending commands to the camera easier. Run sendCommand.

sendCommand &lt;command&gt; &lt;command param&gt; [args]

Commands:

autoFFC &lt;setting&gt;        
set the auto FFC setting on Boson. Valid values are "true" or "false"

pixelFormat &lt;setting&gt;  
set the pixel format. Valid values are "mono_8" and "mono_16" for Boson and, additionally, "color_8" for BlackFly

Options:

-n &lt;namespace&gt; DEFAULT flir_boson

Specifies a namespace for the topic

-s &lt;sub-namespace&gt;

Specifies a sub-namespace for the topic. Typically left or right
e.g. topic is /flir_boson/left/pixel_format then run sendCommand 
pixelFormat mono_8 -n flir_boson -s left
