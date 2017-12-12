1. 考虑可行性：
	1．s上的加速度， 速度
		a_max_greaking < s'' < a_max_accel
	2. d上的加速度  |d''| < ay
		tanθ = L/R  ->  曲率k = tanθ/L  -> k_max = tan(θ_max) / L
		L是车轮轴线之间的距离，R是圆半径 （曲率ki = deltaφ / deltaX （φ为两点切线角，x为两点距离））

2. 考虑优化
	1. jerk优化见 QuinticPolynomialSolver/readme.txt
	2. cost函数要考虑 jerk， 车道中心距离，障碍物距离，时间等。
		balance这些cost权值是难点。

3. 这个exercise中生成轨迹时使用了高斯分布生成一组轨迹点，然后再计算每条轨迹点的cost，使用cost最小的那条作为最终轨迹点。

