include "common/hokuyo.inc"
include "common/camera.inc"

define person position
(
	size [0.30 0.5 1.7]
  origin [0.0 0.0 0 0]

	mass 70.0 
	
	drive "omni"
	# Report error-free position in world coordinates
	localization "gps"
	localization_origin [ 0 0 0 0 ]

	color "red"

	hokuyo( pose [0.25 0 -1.6 0] )

	block (
		points 6
		point[0] [  0.1  0.17 ]
		point[1] [  0.   0.24 ]
		point[2] [ -0.1  0.17 ]
		point[3] [ -0.1  0.1  ]
		point[4] [  0.   0.02 ]
		point[5] [  0.1  0.1 ]
		z [ 0 0.7 ]
	)

	block (
		points 6
		point[0] [  0.1 -0.1 ]
		point[1] [  0.  -0.02 ]
		point[2] [ -0.1 -0.1 ]
		point[3] [ -0.1 -0.17 ]
		point[4] [  0.  -0.24 ]
		point[5] [  0.1 -0.17 ]
		z [ 0 0.7 ]
	)

	block (
		points 6
		point[0] [  0.15  0.   ]
		point[1] [  0.1  0.25 ]
		point[2] [ -0.1  0.25 ]
		point[3] [ -0.15  0.   ]
		point[4] [ -0.1 -0.25 ]
		point[5] [  0.1 -0.25 ]
		z [ 0.7 1.45 ]
	)

	block (
		points 6
		point[0] [  0.10  0.   ]
		point[1] [  0.02  0.09 ]
		point[2] [ -0.07  0.09 ]
		point[3] [ -0.11  0.   ]
		point[4] [ -0.07 -0.09 ]
		point[5] [  0.02 -0.09 ]
		z [ 1.45 1.7 ]
	)
)


