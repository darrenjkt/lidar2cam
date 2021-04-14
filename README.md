# lidar2cam
Projection of lidar pointcloud into a camera image for ROS. This code currently works but runs very slow at 1 FPS for every image. 

Edit the image and pointcloud topic in the launch file. If image topic is `/gmsl/A0` then this will assume your image and camera info are `/gmsl/A0/image_color` and `/gmsl/A0/camera_info`

You can run this with: 
```
roslaunch lidar2cam lidar2cam.launch
```
