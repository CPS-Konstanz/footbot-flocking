# Start up

## Launch simulation environment

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/YOUR_ARGOS3_ROS2_BRIDGE_PATH/lib
export ARGOS_PLUGIN_PATH=/YOUR_ARGOS3_ROS2_BRIDGE_PATH/lib
argos3 -c flocking.argos
```

## Launch controller node
```bash
source install/setup.bash
ros2 launch flocking flocking.launch.py
```

## Launch manual test robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=bot0/cmd_vel
```

## Demo of SDRM

![NODE_GRAPH](picture/node_grapg.png)