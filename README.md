# tram_ws
双电机（转向、驱动）的控制 sil 仿真

依赖项：
ros noetic及其相关的依赖包
python3.8
ubuntu20

编译：

catkin_make

运行：
启动可视化界面或者终端命令运行的形式

## 可视化界面
修改scenario_gui.py：

<img width="401" height="51" alt="image" src="https://github.com/user-attachments/assets/a0f2bb77-8187-4ddb-a2a2-83a38175a244" />

改为正确的路径，然后启动：

source devel/setup.bash

rosrun tram_bringup scenario_gui.py

<img width="1338" height="902" alt="image" src="https://github.com/user-attachments/assets/e93ef4e0-562a-4a18-bb68-14ec8e9c7a40" />

或者终端的形式启动，以下每个 launch 的启动都需要提前 source 一下：
source devel/setup.bash

### 6.1 标准站点停靠

roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true

<img width="1768" height="1172" alt="image" src="https://github.com/user-attachments/assets/2778664b-4d67-4df8-ab1f-9dad7486c32e" />

### 6.2 高速站点停靠

roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true
### 6.3 短站台停靠

roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true
### 6.4 道岔换道

roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true

### 6.5 180° 掉头
roslaunch tram_bringup sim_pi_lqr.launch scenario:=standard_stop rviz:=true

