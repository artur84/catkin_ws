#ifndef VOICE_HANDLER_H_
#define VOICE_HANDLER_H_

#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <ros/ros.h>

namespace ui_local_planner
{

class VoiceHandler
{
public:

	VoiceHandler();
	~VoiceHandler();

	/**
	 * @brief  Init the subscriber to read the input from user
	 */
	virtual void initReader();

	/**
	 * @brief  reads the input from the user
	 * @param input_voice, Twist() reference to the variable where we want store the input voiceocity
	 */
	virtual void read(std_msgs::String& input_voice);



private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber input_voice_sub_; 	//The ROS subscriber to the voice command given by the user
	std_msgs::String input_voice_;
	/**
	 * @brief  Callback for receiving dirocity data
	 * @param msg, Twist(), The dirocity input comming from user
	 */
	void inputVoiceCb(const std_msgs::String::ConstPtr & msg);


};

} /* namespace base_local_planner */
#endif /* VOICE_HANDLER_H_ */
