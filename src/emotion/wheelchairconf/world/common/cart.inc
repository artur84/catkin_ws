include "common/hokuyo.inc"
include "common/camera.inc"

define cart position
(
	size [0.7 0.6 0.33]
	origin [-0.18 0.0 0 0]

	mass 0.0 

	drive "diff"

	# Report error-free position in world coordinates
	localization "gps"
	localization_origin [ 0 0 0 0 ]

	color "blue"

	hokuyo( pose [0.25 0 0 0] )

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
)


