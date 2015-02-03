#a: this robot
#c: object

#position
geometry_msgs/Point a
geometry_msgs/Point c

#radius
float32 ra
float32 rc

#velocity
geometry_msgs/Point va
geometry_msgs/Point vc

#velocity obstacle:
geometry_msgs/Point tp1
geometry_msgs/Point tp2
geometry_msgs/Point apex

#current target
geometry_msgs/Point target

#new velocities
	#velocity towards target, made smaller if its near, not bigger then vmax
	geometry_msgs/Point desired_velocity 

	#new velocity vector pointing towards desired_velocity from current velocity
	geometry_msgs/Point difference_velocity

	#difference velocity rotated to avoid collisions
	geometry_msgs/Point avoidance_velocity

	#requested velocity
	geometry_msgs/Point requested_velocity

#new position
geometry_msgs/Point requested_position