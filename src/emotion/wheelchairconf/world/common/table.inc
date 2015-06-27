include "common/hokuyo.inc"

define table position
(
	size [1.0 1.0 0.9]
  origin [0.0 0.0 0 0]

	mass 30.0 
	
	drive "diff"

	# Report error-free position in world coordinates
	localization "gps"
	localization_origin [ 0 0 0 0 ]

	color "red"
	hokuyo( pose [0.0 0.0 0.0 0] )

	
    #First leg of the table
	block (
		points 4
		point[0] [  0.1  0.1 ]
		point[1] [  0.2  0.1 ]
		point[2] [  0.2  0.2 ]
		point[3] [  0.1  0.2  ]
		z [ 0 0.8 ]
	)

    block (
		points 4
		point[0] [  0.8  0.1 ]
		point[1] [  0.9  0.1 ]
		point[2] [  0.9  0.2 ]
		point[3] [  0.8  0.2  ]
		z [ 0 0.8 ]
	)
	
	block (
		points 4
		point[0] [  0.8  0.8 ]
		point[1] [  0.9  0.8 ]
		point[2] [  0.9  0.9 ]
		point[3] [  0.8  0.9  ]
		z [ 0 0.8 ]
	)
	
	block (
		points 4
		point[0] [  0.1  0.8 ]
		point[1] [  0.2  0.8 ]
		point[2] [  0.2 0.9 ]
		point[3] [  0.1  0.9  ]
		z [ 0 0.8 ]
	)
	#Upper part of the table
	block (
		points 4
		point[0] [  0.0  0.0 ]
		point[1] [  1.0  0.0 ]
		point[2] [  1.0  1.0 ]
		point[3] [  0.0  1.0  ]
		z [ 0.8 0.9 ]
	)
)


