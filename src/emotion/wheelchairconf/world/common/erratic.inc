# create a robot
define erratic position
(
  # actual size
  size [0.40 0.37 0.3]

  # the pioneer's center of rotation is offset from its center of area
  origin [0.0 0.0 0 0]

  # draw a nose on the robot so we can see which way it points
  gui_nose 1

  # estimated mass in KG
  mass 12.0 
	
  # differential steering model
  drive "diff"
)


