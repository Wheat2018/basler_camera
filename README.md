# basler_camera
basler相机ros节点

安装pylon库，[Linux版下载地址](https://www.baslerweb.com/cn/sales-support/downloads/software-downloads/pylon-5-0-12-linux-x86-64-bit-debian/)。

[Windows版下载地址](https://www.baslerweb.com/cn/sales-support/downloads/software-downloads/pylon-6-1-1-windows/)

[其他版本](https://www.baslerweb.com/cn/sales-support/downloads/software-downloads/#type=pylonsoftware;series=baslerace;os=all;version=all)

确保pylon库安装在/opt/pylon5目录下，否则项目依赖lib目录及include目录需要对应修改。

添加环境变量：

$ echo "export LIBRARY_PATH=$LIBRARY_PATH:/opt/pylon5/lib64" >> ~/.bashrc

$ echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/pylon5/lib64" >> ~/.bashrc

source ~/.bashrc
