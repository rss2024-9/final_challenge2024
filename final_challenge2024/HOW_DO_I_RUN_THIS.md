# Simulator
## Localization
In this order, launch localization node first:
```bash
ros2 launch localization localize.launch.xml
```

Then launch the simulator:
```bash
ros2 launch racecar_simulator simulate.launch.xml
```

## City Stopping Controllers
```bash
ros2 launch final_challenge2024 city_stopping_controller.launch.xml
```