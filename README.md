# Projection of LiDAR pointcloud to Image
Projection of lidar pointcloud into a camera image with python in ROS. 

The code takes each pointcloud point and distorts it to fit a distorted camera image. The code currently works but needs optimization (1 FPS).

## Getting started
Edit the image and pointcloud topic in the launch file. 

Note: if your image topic is `/gmsl/A0` then this code will assume your image and camera info are `/gmsl/A0/image_color` and `/gmsl/A0/camera_info`. You can modify that below.
https://github.com/darrenjkt/lidar2cam/blob/11a85f9a7729d3e2efee0e5053053238c117ea29/src/lidar2cam.py#L115-L116

To run the ros python script, use the following command after editing with the topics
```
roslaunch lidar2cam lidar2cam.launch
```
