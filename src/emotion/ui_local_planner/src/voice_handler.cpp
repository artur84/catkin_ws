#include <ui_local_planner/voice_handler.h>

namespace ui_local_planner
{

VoiceHandler::VoiceHandler()
{
}

VoiceHandler::~VoiceHandler()
{
}

void VoiceHandler::initReader()
{
	input_voice_sub_ = nodeHandle.subscribe("recognizer/output", 1,
			&VoiceHandler::inputVoiceCb, this);
}

void VoiceHandler::read(std_msgs::String& input_voice)
{
	input_voice = input_voice_;
}

void VoiceHandler::inputVoiceCb(const std_msgs::String::ConstPtr& msg)
{
	input_voice_.data =msg->data;
}

} /* namespace ui_local_planner */
