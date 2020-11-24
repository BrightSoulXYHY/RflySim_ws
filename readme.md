## 使用说明
1. 创建工作空间
``` 
mkdir ~/RflySim_ws
cd ~/RflySim_ws
```

2. 下载本仓库
```
git clone https://github.com/KennethYangle/RflySim_ws.git
mv RflySim_ws src
```

3. 编译
```
catkin_make
```

4. 刷新ROS环境变量
```
# USER_NAME替换为自己的用户名
source /home/USER_NAME/RflySim_ws/devel/setup.bash
# 或者把这句话加在~/.bashrc，然后重开一个终端，就不用每次都执行上面这句了
```

5. 运行单目或双目图像
```
# 确保仿真环境正在运行，向本机发送数据。单目或双目运行其中一个就好
# 单目
roslaunch rflysim_ros_pkg cameras.launch
# 双目
roslaunch rflysim_ros_pkg stereo.launch
```

6. 其他节点接收消息。默认情况下图像话题名为/camera/left和/camera/right。可以使用下面语句快速查看。
```
rqt_image_view /camera/left
```