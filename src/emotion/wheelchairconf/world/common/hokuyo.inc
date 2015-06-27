define hokuyo ranger
(
	sensor(
		range [ 0.02 5.6 ]
		fov 240
		samples 667
	)

	size [ 0.075 0.06 0.06 ]

	color "black"
	block(
		points 6
		point[0] [  0.0375  0.015 ]
		point[1] [  0.0225  0.03 ]
		point[2] [ -0.0375  0.03 ]
		point[3] [ -0.0375 -0.03 ]
		point[4] [  0.0225 -0.03 ]
		point[5] [  0.0375 -0.015 ]
		z [ 0 0.06 ]
	)
)

