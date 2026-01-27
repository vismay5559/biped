to launch everything - bash ~/ros2_ws/src/biped/biped_bringup/scripts/biped_280_gazebo.sh
to run the launch file - ros2 launch biped_gazebo biped_gazebo.launch.py
to run com visual - ros2 run biped_system_tests com_vis.py and add visuals of topic /com in rviz gui
to run biped stand controller -  ros2 run biped_system_tests biped_stand_controller.py
to run any python file - ros2 run biped_system_tests [python file]
to visualise the urdf in rviz - ros2 launch urdf_tutorial display.launch.py model:=/home/vismay/ros2_ws/src/biped/biped_description/urdf/robots/biped.xacro    [replace the path with ur files path]
