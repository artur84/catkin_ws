define kinect_laser ranger
(
sensor (
  fov 57.0
  samples 640
  # generic model properties
  range [.4 4.0]
)
 size [ 0.08 0.2 0.04 ]
 color "black"
 block(
		points 4
		point[0] [ 0.04  0.1  ]
		point[1] [ 0.04  -0.1 ]
		point[2] [ -0.04 -0.1 ]
		point[3] [ -0.04 0.1 ]
		z [ 0 0.04 ]
	)
 
)