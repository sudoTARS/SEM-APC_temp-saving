ros2 launch carla_shell_bridge main.launch.py

ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

ros2 launch carla_waypoint_publisher carla_waypoint_publisher.launch.py

ros2 run shell_simulation example_control.py

ros2 run carla_ad_agent local_planner

export PYTHONPATH=$PYTHONPATH:/tmp/PythonAPI/carla