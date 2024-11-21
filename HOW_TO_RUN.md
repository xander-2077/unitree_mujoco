# How to RUN MuJoCo simulator

The path of this repo is 
```shell
/home/zdj/Quadruped/go2/unitree/unitree_mujoco
```

## C++ simulator

Start simulation:
```
cd /simulate/build
./unitree_mujoco
```

Run `./test` to get the robot's pose and position information.

The config file is in `/simulate/config.yaml`.

## Python simulator

Firstly, activate venv: 
```
source /home/zdj/Quadruped/go2/unitree/unitree_sdk2_python/venv/bin/activate
```

Run 
```
cd /simulate_python
python ./unitree_mujoco.py
```
to open mujoco simulator.

Run `python ./test/gamepad_test.py` to use joystick.

Run `python ./test/test_unitree_sdk2.py` to get the robot's pose and position information.

The config file is in `config.yaml`.

## ROS2
	
Set up the environment varible:
```
source /home/zdj/Quadruped/go2/unitree/unitree_ros2/setup_local.sh
export ROS_DOMAIN_ID=1
```

Test:
```
cd example/ros2
./install/stand_go2/bin/stand_go2
```
	
## Body
['world', 'base_link', 'FL_hip', 'FL_thigh', 'FL_calf', 'FL_foot', 'FR_hip', 'FR_thigh', 'FR_calf', 'FR_foot', 'RL_hip', 'RL_thigh', 'RL_calf', 'RL_foot', 'RR_hip', 'RR_thigh', 'RR_calf', 'RR_foot', 'soccer_ball']