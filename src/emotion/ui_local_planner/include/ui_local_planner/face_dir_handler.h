#ifndef USER_DIR_HANDLER_H_
#define USER_DIR_HANDLER_H_

#include <tf/transform_datatypes.h>
#include <ui_local_planner/user_vel_handler.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

namespace ui_local_planner
{

class FaceDirHandler: public UserVelHandler
{
public:

	FaceDirHandler();
	~FaceDirHandler();
};

} /* namespace base_local_planner */
#endif /* USER_DIR_HANDLER_H_ */
