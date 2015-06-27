# create a robot
define agent position
(
  # actual size
  size [0.58 0.58 0.8]

  # the pioneer's center of rotation is offset from its center of area
  origin [0.0 0.0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 40.0 
	
  # differential steering model
  drive "diff"

	color "blue"
	block (
		points 4
		point[0] [ -0.29 -0.29 ]
		point[1] [  0.29 -0.29 ]
		point[2] [  0.29  0.29 ]
		point[3] [ -0.29  0.29 ]
		z [ 0 0.35 ]
	)

	block (
		points 4
		point[0] [ -0.29 -0.29 ]
		point[1] [  0.0   -0.29 ]
		point[2] [  0.0    0.29 ]
		point[3] [ -0.29  0.29 ]
		z [ 0.35 0.8 ]
	)
)


