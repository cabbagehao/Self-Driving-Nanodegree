车辆行为决策
	cpp和py版本里面是我实现的基本方法变道/保持车道等行为。
	state-mechine版本是通过状态机的方式，加上cost函数，去实现了一个更精确的行为决策。

note：
	此模拟忽略了很多因素。 只是为了突出行为决策的部分。
	比如变道是瞬间变过去的，车辆大小，方向等都未建模。

1. 考虑可行性：  
	1．s上的加速度， 速度  
    1. a_max_greaking < s'' < a_max_accel    
	2. d上的加速度  |d''| < ay    
		1. tanθ = L/R  ->  曲率k = tanθ/L  -> k_max = tan(θ_max) / L     
		2. L是车轮轴线之间的距离，R是圆半径 （曲率ki = deltaφ / deltaX （φ为两点切线角，x为两点距离））  

2. 考虑优化   
	1. jerk优化见 QuinticPolynomialSolver/readme.txt   
	2. cost函数要考虑 jerk， 车道中心距离，障碍物距离，时间等。   
		balance这些cost权值是难点。
	
