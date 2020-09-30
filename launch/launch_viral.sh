catkin_make -C /home/tmn/AirSim/ros; \
catkin_make -C /home/tmn/dev_ws; \

bash ~/EnvAirSim/Building_99/Building_99.sh \
&
sleep 5; \
roslaunch airsim_pathplanner viral.launch \