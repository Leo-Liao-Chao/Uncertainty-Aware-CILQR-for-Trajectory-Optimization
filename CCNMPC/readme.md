In CCNMPC directory:

make:

```
catkin_make
```

Start:

1. Carla:
   ```
   conda activate Carla &&
   cd CARLA && 
   SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
   ```
2. ros-bridge: change start position
   ```
   conda deactivate &&
   cd carla-ros-bridge/catkin_ws_changed/ &&
   source devel/setup.bash &&
   roslaunch carla_ad_demo carla_ad_demo.launch
   ```
3. vehiclepub:
   ```
   conda deactivate && source devel/setup.bash
   rosrun vehiclepub main.py
   ```
4. ccnmpc:
   ```
   roslaunch nmpc Experiment.launch
   ```
5. record:
   ```
   rosbag record -O 1 /experiment

   ```
