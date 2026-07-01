# imu校准
sudo systemctl stop pug_bringup.service
roslaunch pug_peripherals imu_calibrate.launch

# 查看校准效果
sudo systemctl stop pug_bringup.service
roslaunch pug_peripherals imu.launch debug:=true

rosrun pug_tutorial demo01_trot_gait.py
rosrun pug_tutorial demo02_amble_gait.py
rosrun pug_tutorial demo03_walk_gait.py
rosrun pug_tutorial demo04_stay.py
rosrun pug_tutorial demo05_trot_turn.py

rosrun pug_tutorial demo01_kinematics.py
rosrun pug_tutorial demo02_horizontal_move.py
rosrun pug_tutorial demo03_omnidirection_move.py
rosrun pug_tutorial demo04_move_speed_adjustment.py
rosrun pug_tutorial demo05_move_height_adjustment.py

rosrun pug_tutorial demo01_gravity_center_adjustment.py
rosrun pug_tutorial demo02_wave_body.py
rosrun pug_tutorial demo03_self_balance01.py
rosrun pug_tutorial demo04_self_balance02.py

rosrun pug_example color_detect_demo.py
rosrun pug_example color_tracking_demo.py
rosrun pug_example kcf_tracking.py

rosrun pug_example hand_detect.py
rosrun pug_example gesture_control.py
rosrun pug_example finger_track.py
rosrun pug_example pose_control.py

rosrun pug_example visual_patrol_demo.py
rosrun pug_example kick_ball_demo.py
rosrun pug_example negotiate_stairs_demo.py

rosrun pug_example yolov5_node.py

python3 /home/hiwonder/pug/src/pug_tutorial/scripts/adapater_board_course/led.py
python3 /home/hiwonder/pug/src/pug_tutorial/scripts/adapater_board_course/button.py

# slam建图（含app建图）
sudo systemctl stop pug_bringup.service
roslaunch pug_slam gmapping.launch

# 导航（含app）
roslaunch pug_navigation navigation.launch map:=map_01

# urdf显示
sudo systemctl stop pug_bringup.service
roslaunch pug_description display.launch

# gazebo仿真
sudo systemctl stop pug_bringup.service
# 虚拟机
roslaunch pug_description gazebo.launch

# jetson
roslaunch pug_description sim_base.launch

