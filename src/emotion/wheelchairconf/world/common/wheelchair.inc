include "common/sick.inc"
include "common/kinect_laser.inc"
define wheelchair position
(
      # actual size in meters
      # (need height so it shows up on laser scans)
      size [1.3 0.65 1.2]
      origin [0 0 0 0]
      gui_nose 1            # Draw a nose on the robot so we can see which way it points
      drive "diff"          # Differential steering model.
      obstacle_return 1     # Can hit things.
      mass 130.0            # Robot mass
      color "gray50"           # Default color.
      # Report error-free position in world coordinates
      #localization "gps"
      #localization_origin [ 0 0 0 0 ]
      sick_laser( pose [0.2689 0 -1.1 0 ] )
      #kinect_laser( pose [-0.6 0 .1 0 ] ) #uncomment this/and comment sick_laser if you want to simulate the upper kinect of the wheelchair

      # Base
      block (
	    points 6
	    point[0] [  0.0   0.0   ]
	    point[1] [  1.1  0.0 ]
	    point[2] [1.3 0.2 ]
	    point[3] [ 1.3 0.45   ]
	    point[4] [ 1.1 0.65 ]
	    point[5] [ 0.0 0.65  ]
	    z [ 0.1 0.15 ]
	    )

	   # Sit
      block (
	    points 4
	    point[0] [  0.3   0.1   ]
	    point[1] [  1.0  0.1 ]
	    point[2] [1.0 0.55 ]
	    point[3] [ 0.3 0.55   ]

	    z [ 0.3 0.6 ]
	    )


      #Back Support
      block (
		  points 4
		  point[0] [  0.15  0.1  ]
		  point[1] [  0.3  0.1  ]
		  point[2] [  0.3  0.55 ]
		  point[3] [  0.15  0.55 ]
		  z [ 0.5 1.2 ]
	    )
     #Right arm support
      block (
        points 4
        point[0] [  0.4  0.0  ]
        point[1] [  0.8  0.0  ]
        point[2] [  0.8  0.1 ]
        point[3] [  0.4  0.1 ]
        z [ 0.7 0.8 ]
        )

      #Left arm support
      block (
        points 4
        point[0] [  0.4  0.55  ]
        point[1] [  0.8  0.55  ]
        point[2] [  0.8  0.65 ]
        point[3] [  0.4  0.65 ]
        z [ 0.7 0.8 ]
        )
     #Right Wheel
     block (
        points 4
        point[0] [  0.4  0.0  ]
	    point[1] [  0.8  0.0  ]
	    point[2] [  0.8  0.1 ]
	    point[3] [  0.4  0.1 ]
	    z [ 0.0 0.4 ]
	    )
	#Left wheel
	block (
        points 4
        point[0] [  0.4  0.55  ]
	    point[1] [  0.8  0.55  ]
	    point[2] [  0.8  0.65 ]
	    point[3] [  0.4  0.65 ]
	    z [ 0.0 0.4 ]
	    )





)

