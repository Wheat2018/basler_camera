# basler_camera
basler相机ros节点

# 安装pylon库

- [Linux版下载地址](https://www.baslerweb.com/cn/sales-support/downloads/software-downloads/pylon-5-0-12-linux-x86-64-bit-debian/)。

- [Windows版下载地址](https://www.baslerweb.com/cn/sales-support/downloads/software-downloads/pylon-6-1-1-windows/)

- [其他版本](https://www.baslerweb.com/cn/sales-support/downloads/software-downloads/#type=pylonsoftware;series=baslerace;os=all;version=all)


# 添加环境变量

- 确保pylon库安装在/opt/pylon5目录下，否则项目依赖lib目录及include目录需要对应修改。

echo "export LIBRARY_PATH=\\$LIBRARY_PATH:/opt/pylon5/lib64" >> ~/.bashrc

echo "export LD_LIBRARY_PATH=\\$LD_LIBRARY_PATH:/opt/pylon5/lib64" >> ~/.bashrc

source ~/.bashrc

# 运行示例

- 将节点源文件夹放在ros工作空间下的src目录下，于ros工作空间目录执行catkin_make，编译节点。

- 建议将工作空间获取脚本命令添加入当前用户的.bashrc内容，以便每次打开新的Shell时自动获取所有节点（以下指令假设ros工作空间为~/catkin_ws/)：

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

- 执行basler相机图像捕获节点

rosrun basler_camera basler_acA1600 

- 执行basler相机图像监听节点（示例）

rosrun basler_camera basler_acA1600_listener_demo
