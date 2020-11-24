## 使用说明
1. 创建工作空间
``` 
mkdir ~/Rflysim_ws
cd ~/Rflysim_ws
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

4. 运行单目或双目图像
```
# 确保仿真环境正在运行，向本机发送数据。单目或双目运行其中一个就好
# 单目
roslaunch rflysim_ros_pkg cameras.launch
# 双目
roslaunch rflysim_ros_pkg stereo.launch
```

5. 其他节点接收消息。默认情况下图像话题名为/camera/left和/camera/right。可以使用下面语句快速查看。
```
rqt_image_view /camera/left
```