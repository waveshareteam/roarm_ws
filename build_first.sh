cd /home/ws/roarm_ws
colcon build --packages-select roarm_msgs 
colcon build --packages-select moveit_servo 
colcon build --packages-select rviz_marker_tools 
colcon build --packages-select moveit_task_constructor_msgs 
colcon build --packages-select moveit_task_constructor_core 
colcon build --packages-select moveit_task_constructor_capabilities 
colcon build --packages-select moveit_task_constructor_visualization 
colcon build --packages-select roarm_moveit_cmd 
colcon build --packages-select roarm_moveit_ikfast_plugins 
colcon build --packages-select roarm_moveit_mtc_demo 
colcon build --packages-select roarm_moveit_servo 
colcon build --packages-select roarm_description roarm_driver roarm_moveit --symlink-install 
echo "source /home/ws/roarm_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc 
