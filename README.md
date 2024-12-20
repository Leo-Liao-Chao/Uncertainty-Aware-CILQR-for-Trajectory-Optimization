make:

```
source /opt/intel/oneapi/setvars.sh
```

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

---

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
4. map_engine:

   ```
   rosrun rqt_reconfigure rqt_reconfigure
   roslaunch map_engine map_engine.launch
   ```
5. cilqr:

   ```
   source /opt/intel/oneapi/setvars.sh 
   roslaunch ilqr Experiment.launch
   ```
6. record:

   ```
   rosbag record -O 1 /experiment

   ```
