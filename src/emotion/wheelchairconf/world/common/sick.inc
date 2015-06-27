define sick_laser ranger
(
sensor (
  fov 180.0
  samples 181
  # generic model properties
  range [0.0 16.0]
)
 size [ 0.14 0.14 0.1 ]
 color "blue"
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

# extends sicklaser to add nice-looking but relatively expensive geometry
define fancysicklaser sicklaser
(
  # bottom
  block(
    points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0 0.02 ]
  )

  # back
  block(
    points 4
    point[0] [ -0.078 -0.077 ]
    point[1] [ -0.02  -0.077 ]
    point[2] [ -0.02   0.077 ]
    point[3] [ -0.078  0.077 ]
    z [0 0.21 ]
  )

  # top
  block( points 4
    point[0] [ -0.02 -0.077 ]
    point[1] [  0.078 -0.077 ]
    point[2] [  0.078  0.077 ]
    point[3] [ -0.02  0.077 ]
    z [0.12 0.21 ]
  )

  # laser bit
  block( points 4
    point[0] [ -0.02 -0.05 ]
	 point[1] [  0.06 -0.05 ]
	 point[2] [  0.06  0.05 ]
	 point[3] [ -0.02  0.05 ]
	 z [0.02 0.12 ]
   color "gray10"
  )
)



