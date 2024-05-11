# Simulation
Launch the path planner:
```bash
ros2 run vehicular_manslaughter path_planner
```
Launch the path follower:
```bash
ros2 run path_planning trajectory_follower --ros-args -p drive_topic:="/drive"
```
Launch the simulator:
```bash
ros2 launch racecar_simulator simulate.launch.xml
```