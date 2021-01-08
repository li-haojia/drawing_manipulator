# 语音自主导航绘图机器人

本系统在 Ubuntu 16.04 + ros kinetic下测试通过  
机械臂控制采用move_it  
导航控制采用move_base  
## 绘图部分
svg图片边缘轮廓->转为gcode->解析gcode ->运动学逆解执行
### 运行方法
#### 仿真测试
```bash
# 在第一个终端打开
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch robot_ip:=127.0.0.1
# 在第二个终端打开
roslaunch aubo_gazebo aubo_i5_gazebo_control.launch
# 在第三个工作空间打开 并切换到工作空间

```
1. 原点设定

### 文件结构
`aubo_robot` 奥博机械臂驱动  

`industrial_core` 奥博机械臂驱动依赖 

`draw_core` 绘图部分核心代码

- `arm_controller.py` 机械臂控制核心
- `gcode_draw.py` gcode解析与绘制
- `gcode_excute` gcode指令与机械臂控制的中间接口文件
- `svg_convert` svg转gcode
- `py_svg2gcode` svg转Gcode核心代码库
  - `lib/shapes.py` svg形状类型解析
  - `lib/cubicsuperpath.py` 曲线插值
