## 遨博aubo-i7机器人moveIt2仿真
确保你的ROS2和MoveIt2环境[Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon)
并在环境变量中source相应的setup.bash
```
mkdir -p ~/ws_moveit/src
git clone https://github.com/xieyaowei/aubo_i7_moveit_simulation.git
cd ..
colcon build
ros2 launch aubo_moveit_config demo.launch.py
```
打开一个新的终端
```
cd ~/ws_moveit2
source install/setup.bash
ros2 run trajectory_planning trajectory_planning
```
