There are three kinds of vehicles:
1. Keep lane if the road is clear.
	If there's no vehicle  in front of the current lane in the detection distance of 30m, then the maximum speed in the current drive.
2. Encounter congestion, keep in line with the car.
	If the vehicle in front of the current lane in 30m, and the adjacent lane gap is not enough, then slow down the same speed with the front of the vehicle.

3. Encounter congestion, change lane.
	If the vehicle in front of the current lane in 30m, and the adjacent lane has enough space, then immediately change lane.



How to generate the trajectory:
1. Each time 50 points are predicted as the trajectory of the vehicle

2. The 50 points consist of two parts:
	1. The previous generation of previous_path_x's unprocessed points.
	2. newly generated trajectory.

3. The newly generated trajectory uses the spline library to fit a curve. The spline used points consist of two parts:
	1. The last time the previous_path was calculated after the last two points
	2. Take 3 points on the calculated lane. The d value of the three points is the center of the calculated lane, and the s values are  55m 65m and 75m in front of the current vehicle.
	3. Transform these points to the map global coordinate system.

	In this way, both to ensure that the newly generated trajectory and the previous track of the fit, but also to ensure that there will not be too much change after the jerk.


How to change lane:
1. Calculate the current lane of the vehicle and two adjacent lanes (probably only one, such as the right side of the vehicle).
2. Calculate the distance between the nearest two cars in the adjacent lane.
3. If there is enough spacing, then immediately change. If there is no spacing, slow down the same speed as the front vehicle and wait for the chance to change.



