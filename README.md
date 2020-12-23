# raspicam_ros
Enables a Raspicam on ROS Noetic at Ubuntu 20.04. Tested on a Raspberry Pi 3B+.

## Before you use this package:
Adjust your Raspberry Pi 3B+ by enabling the camera connected on the camera socket. To do that:

##### Edit *config.txt*:
At the path */boot/firmware*  and edit the file *config.txt*:
`$ sudo nano /boot/firmware/config.txt` then  add a line `start_x=1` in the section dedicated to the Raspberry Pi 3B+ changes. The section starts below the **[rpi3]** line.

Save the file then do: `sudo apt upgrade` 

##### Install OpenCV: 
Follow the instructions in https://computingforgeeks.com/how-to-install-opencv-on-ubuntu-linux/#ex1

### Install o CV-Bridge:
`$ sudo apt-get install ros-noetic-cv-bridge`

