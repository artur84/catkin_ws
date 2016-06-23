#This message contains the goal structure needed by the user intentions 
#inference algorithm.
#pose_stamped is the pose of the goal in the map
geometry_msgs/Pose2D pose
#proba is the posterior probability value for this goal.
float64 proba
#string, parent of this goal, in a semantic map this will be the parent of the current goal 
string parent_goal
#tag is a one word description of the goal to represente the semantics as chair, 
#table, door, person. etc.
string tag
#id is a number to differentiate between different goals of the same type (tag)
uint32 id
