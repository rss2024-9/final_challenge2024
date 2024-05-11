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

# Real Life
Launch the camera:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```
If it errors with something about a display, run `unset DISPLAY` before this command

Launch localize:
```bash
ros2 launch localization real_localize.launch.xml
```
You may need to do this first:
```bash
cp /root/racecar_ws/src/lab5/localization/real_params.yaml /root/racecar_ws/install/localization/share/localization/
```

Launch the simulator:
```bash
ros2 launch racecar_simulator localization_simulate.launch.xml
```

## Visualizing Stuff
- Launching noVNC:
    - Locally, *not* on the racecar: `ssh -L 6081:localhost:6081 racecar@192.168.1.74`
    - Open [noVNC](http://localhost:6081/vnc.html?resize=remote) in your web browser
- Open a terminal, enter `rqt` and go to `Plugins > Visualization > Image View`
- Change the subscription topic (left-most dropdown) to `/zed/zed_node/left/image_rect_color`
    - You may need to hit "refresh topics" (button to the right of the dropdown) if it's not working