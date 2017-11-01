![]() 
![pid_control_hand_parameter.png](手动设定参数结果) 
[image1]: ./imgs/pid_control.png "twiddle程序结果"
![alt text](./imgs/pid_control.png)

robot.py: 一个简单的机器人类，定义了move动作及其他状态
pid_control: 使用手动设定的PID参数去控制机器人的路径，让其尽快平滑地靠近目标路径。
twiddle.py:  使用了twiddle程序去自动寻找最佳参数。
