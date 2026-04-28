source /opt/ros/jazzy/setup.bash 
source ~/ros_venv/bin/activate
source install/setup.bash


ros2 bag play --rate 0.4  ~/Self-Built\ Dataset\ Parsing/ptp_pps_time_synchroize/ros2_data/rosbag2_2026_02_28-15_44_55
 ros2 launch sync_export sync_export.launch.py