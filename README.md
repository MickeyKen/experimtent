# experimtent


### 卒 ###
roslaunch ubiquitous_display_description show_ubiquitous_display.launch

roslaunch ubiquitous_display_pantilt start_pantilt.launch

roslaunch experiment_miki prepare.launch

roslaunch experiment_miki exp_ver1.launch

rosrun experiment_miki start_exp.py $(exp_num)

exp_num: 4 ~ 7 , 0 (fin)


==== bag file ====

rosparam set /use_sim_time true

rosbag play --clock file_name.bag


### RM ###

roslaunch ubiquitous_display_bringup ubiquitous_display_model.launch

roslaunch ubiquitous_display_bringup ubiquitous_display_pantilt.launch

roslaunch youbot_driver_ros_interface youbot_driver_for_ud.launch

roslaunch experiment_miki prepare.launch

roslaunch experiment_miki exp_ver2.launch

rosrun experiment_miki control_experience.py

