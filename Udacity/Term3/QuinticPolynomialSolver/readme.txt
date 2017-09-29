需求： 让car从当前位置，运动到指定位置（如变道，掉头等），并且考虑乘坐舒适性。

问题： 实际上就是求一个多项式，拟合出1个轨迹满足位置条件，且满足舒适性要求。

1. 我们要优化什么？
	位移求导是速度，速度变化是加速度。然后是急动度，痉挛度等。 乘客是否舒服实际上取决于Jerk
	postion -> velocity -> acceleration -> Jerk(急动度) -> snap(痉挛度) -> crackle -> pop
	因此我们优化jerk就行了。

2. 如何优化？
	1. 在frenet坐标系，可以单独考虑s和d与时间t的关系，这样比较简单。　d与时间t的计算和下面类似。
	2. 先考虑s与时间的函数s(t)：
		s(t) ∈[0, tf]
		Jerk = s'''(t)
		Total Jerk = ∫s'''(t)^2 dt    // 因为正负jerk我们都要考虑，因此取平方。 找到这个函数最小值就行了。
		由积分公式， m>=6阶时  s的m阶微分/t的m阶微分 = 0，因此s的最高阶设为5阶。
	3. 计算s(t)
		s(t)有6个参数需要确定。 我们有3组数据可以利用： 
			[si, s'i, s''i]  起始点s， 起始点速度， 起始点加速度
			[sf, s'f, s''f]	 目标点s， ..	..
			T		 完成变化所用时间
		设s(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5 则：
			s'(t) = a1 + 2a2*t + 3a3*t^2 + 4a4*t^3 + 5a5*t^4
			s''(t) = 2a2 + 6a3*t + 12a4*t23 + 20a5*t^3
		取t_i = 0,则：
			a2 = 0.5s''(t_i) = s''i;   a1 = s'i;  a0 = si
		因此：
			s(t) = a3*t^3 + a4*t^4 + a5*t^5  + C1        	// C1 = a0 + a1*t + a2*t^2
			s'(t) = 3a3*t^2 + 4a4*t^3 + 5a5*t^4 + C2	// C2 = a1 + 2a2*t 
			s''(t) = 6a3*t + 12a4*t23 + 20a5*t^3 + C3	// C3 = 2a2
		将a3 a4 a5看做未知数, t^3  3f^2等看做是系数，则方程组可看做是矩阵的乘积。　// 因为是求时间Ｔ内的积分，所以t=T是常数。	
		即Ax=B：
		    Ａ << t_cube, t_square*t_square,  t_square*t_cube,
			  3*t_square,4*t_cube, 5*t_square*t_square,
			  6*T, 12*t_square, 20*t_cube;
    
		    B << sf - (si + si_dot*T + 0.5*si_d_dot*T*T), 
			 sf_dot - (si_dot + si_d_dot*T),
			 sf_d_dot - si_d_dot;
		所以　x = A.inverse() * B
		所以s(t)的　后三个参数a3 a4 a5就是ｘ的解。　前３个参数a0 a1 a2之前就算出来了。





