
`robot.py`:&emsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;一个简单的机器人类，定义了move动作及其他状态  
`pid_control`:&emsp;使用手动设定的PID参数去控制机器人的路径，让其尽快平滑地靠近目标路径。  
`twiddle.py`:&emsp;&nbsp;&nbsp;使用了twiddle程序去自动寻找最佳参数。  

twiddle结果：  
![twiddle](./imgs/pid_control.png)  

手动设定参数结果:  
![手动设定参数结果](./imgs/pid_control.png)  

PID组合对比：  
![手动设定参数结果](./imgs/pid_compare.png)  



