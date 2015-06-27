include "common/hokuyo.inc"
include "common/camera.inc"

define robot position
(
	size [0.33 0.33 0.075]
	origin [0.0 0.0 0 0]

	# This block approximates a circular shape of a Robot
	block
	(
		points 16
		point[0] [ 0.225 0.000 ]
		point[1] [ 0.208 0.086 ]
		point[2] [ 0.159 0.159 ]
		point[3] [ 0.086 0.208 ]
		point[4] [ 0.000 0.225 ]
		point[5] [ -0.086 0.208 ]
		point[6] [ -0.159 0.159 ]
		point[7] [ -0.208 0.086 ]
		point[8] [ -0.225 0.000 ]
		point[9] [ -0.208 -0.086 ]
		point[10] [ -0.159 -0.159 ]
		point[11] [ -0.086 -0.208 ]
		point[12] [ -0.000 -0.225 ]
		point[13] [ 0.086 -0.208 ]
		point[14] [ 0.159 -0.159 ]
		point[15] [ 0.208 -0.086 ]
		z [ 0 0.075 ]
	)
	
	hokuyo( pose [0 0 0 0] )

	drive "diff"

	# Report error-free position in world coordinates
	localization "gps"
	localization_origin [ 0 0 0 0 ]

	color "gray50"
)

define cart position
(
	size [0.6 0.7 0.33]
	origin [0.0 0.18 0 0]

	# This block approximates a circular shape of a Robot
	block
	(
		points 8
		point[0] [  0.18  0.25 ]
		point[1] [  0.13  0.3  ]
		point[2] [ -0.18  0.3  ]
		point[3] [ -0.52  0.05 ]
		point[4] [ -0.52 -0.05 ]
		point[5] [ -0.18 -0.3  ]
		point[6] [  0.13 -0.3  ]
		point[7] [  0.18 -0.25 ]
		z [ 0 0.36 ]
	)
	
	hokuyo( pose [0 0 0 0] )

	drive "diff"

	# Report error-free position in world coordinates
	localization "gps"
	localization_origin [ 0 0 0 0 ]

	color "blue"
)


