uint64 timestamp		# time since system start (microseconds)

uint16 att_mode		# attitude modes as defined in ThrustVectoring mode parameter

uint16 manual_orientation	# Manual oriention for the thrust vector

float32 tilt_angle_est		# estimated optimal tilt angle of the robot in rad
float32 tilt_angle_meas		# measured tilt angle of the robot in rad

float32 tilt_direction_est	# estimated optimal tilt direction in local NED frame
float32 tilt_direction_meas	# tilt direction in local NED frame

float32 tilt_roll_est		# estimated optimal roll angle of the robot in rad
float32 tilt_pitch_est		# estimated optimal pitch angle of the robot in rad

