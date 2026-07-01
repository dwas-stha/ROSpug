#!/bin/bash
source /home/hiwonder/.hiwonderrc
sudo service nvargus-daemon restart
roslaunch pug_bringup app_bringup.launch

