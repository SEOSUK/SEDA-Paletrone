![inchworm_illust](https://github.com/SEOSUK/inch_worm/assets/99397827/18774c07-8662-40a9-a0e4-abbc2ccb38eb)

Update Comming Soon,....
```
alias im='cd ~/inch_ws && catkin_make && source ~/inch_ws/devel/setup.bash'

alias iw='source ~/inch_ws/devel/setup.bash'
alias cw='source ~/catkin_ws/devel/setup.bash'
alias inch_stop='source ~/inch_ws/devel/setup.bash && roslaunch dynamixel_workbench_controllers inch_position_control.launch'
alias inch_controller='roslaunch inch_lab preparing_mpc.launch'
alias inch_mpc='roslaunch inch_lab trying_mpc.launch'
alias inch_final='roslaunch inch_lab final_mpc.launch'
alias inch_optitrack='roslaunch optitrack optitrack.launch'
alias inch_tf='roslaunch setting_tf2 tf2_setting.launch'
alias inch_bag='rosrun inch_bag inch_data_logging'
alias inch_bag2='cd ~/inch_ws/src/inch_worm/inch_worm/inch_bag/bag && rosbag record /data_log'

alias pt='sudo chmod 777 /dev/i2c-1 && sudo cpufreq-set -g performance && sudo setserial /dev/ttyU2D2 low_latency && roslaunch FAC_MAV FAC_MAV_launcher.launch'
alias ipt='sudo chmod 777 /dev/i2c-1 && sudo cpufreq-set -g performance && sudo setserial /dev/ttyU2D2 low_latency && roslaunch FAC_MAV FAC_MAV_with_ARM.launch'
alias happy='shutdown -h now'
```
