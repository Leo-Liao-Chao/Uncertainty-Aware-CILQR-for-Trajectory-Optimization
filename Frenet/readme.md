1. Origin:
   * launch:sigma_x,sigma_y,sigma_theta
     1. add rand;
     2. sigma_x,sigma_y,sigma_theta = 0;
   * map_engine:sigma_x,sigma_y,sigma_theta:=0;
2. Expansion:
   * launch:sigma_x,sigma_y,sigma_theta
     1. add rand;
     2. sigma_x,sigma_y,sigma_theta
   * map_engine:sigma_x,sigma_y,sigma_theta:=0;
3. UncertaintyPropagation:
   * launch:sigma_x,sigma_y,sigma_theta
     1. add rand;
     2. sigma_x,sigma_y,sigma_theta
   * map_engine:sigma_x,sigma_y,sigma_theta;

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
   ```
5. ```
   `roslaunch map_engine map_engine.launch`
   ```
6. cilqr:
   ```
   source /opt/intel/oneapi/setvars.sh 
   roslaunch ilqr Experiment.launch
   ```
7. record:
   ```
   rosbag record -O 1 /experiment

   ```
