# experimtent

roslaunch ubiquitous_display_description show_ubiquitous_display.launch 

roslaunch ubiquitous_display_pantilt start_pantilt.launch 

roslaunch experiment_miki prepare.launch

roslaunch experiment_miki exp_miki.launch

rosrun experiment_miki start_exp.py $(exp_num)

exp_num: 4 ~ 7 , 0 (fin)


==== bag file =====
rosparam set /use_sim_time true
rosbag play --clock filename.bag
