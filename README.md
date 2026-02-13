# mini_serial_demo
这是一个简易的mini串口收发控制的demo，目前使用ros1发布的mini的里程计话题，未做ros2的适配。

## 通讯参数
UART的接口为GH1.0的5pin，从螺丝孔那里开始从左到右分别是GND、TRIG、TX、RX、5V，波特率：115200，3.3v电平

## build && run

```
mkdir -p ~/mini_serial_demo
cd ~/mini_serial_demo
git clone https://github.com/Hessian-matrix/mini_serial_demo
catkin_make
source devel/setup.bash
roslaunch serial_demo serial_demo.launch
```

## 运行效果
运行后终端等待用户的按键输入：[ INFO] [1770238508.251035632]: please input command: 1-start, 2-stop, 3-reset, 4-exit, a-switch algo
- 1： 启动算法
- 2： 停止算法
- 3： 重置算法
- 4： 退出程序
- a:  切换算法
