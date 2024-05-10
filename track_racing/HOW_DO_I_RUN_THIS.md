# Simulator
This will likely only work locally. You need the follow dependencies first:
```bash
pip install trimesh
pip install pyrender
pip install "pyglet<2"
```
Then build (`colcon build && source install/setup.bash`) and run these launch files:
```bash
ros2 launch track_racing track_racing_sim.launch.xml
ros2 launch track_racing track_racing.launch.xml
```

# Real Life (TBD)
Launch the camera:
```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed
```
If it errors with something about a display, run `unset DISPLAY` before this command

Launch the track lane detector:
```bash
ros2 launch track_racing track_racing_real.launch.xml
```

## Visualizing Stuff
- Launching noVNC:
    - Locally, *not* on the racecar: `ssh -L 6081:localhost:6081 racecar@192.168.1.74`
    - Open [noVNC](http://localhost:6081/vnc.html?resize=remote) in your web browser
- Open a terminal, enter `rqt` and go to `Plugins > Visualization > Image View`
- Change the subscription topic (left-most dropdown) to `/zed/zed_node/left/image_rect_color`
    - You may need to hit "refresh topics" (button to the right of the dropdown) if it's not working