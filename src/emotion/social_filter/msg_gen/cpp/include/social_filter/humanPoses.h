/* Auto-generated by genmsg_cpp for file /home/arturo/ros_works/roswheelchair/trunk/ros/stacks/inria_wheelchair/social_filter/msg/humanPoses.msg */
#ifndef SOCIAL_FILTER_MESSAGE_HUMANPOSES_H
#define SOCIAL_FILTER_MESSAGE_HUMANPOSES_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "social_filter/humanPose.h"

namespace social_filter
{
template <class ContainerAllocator>
struct humanPoses_ {
  typedef humanPoses_<ContainerAllocator> Type;

  humanPoses_()
  : humans()
  {
  }

  humanPoses_(const ContainerAllocator& _alloc)
  : humans(_alloc)
  {
  }

  typedef std::vector< ::social_filter::humanPose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::social_filter::humanPose_<ContainerAllocator> >::other >  _humans_type;
  std::vector< ::social_filter::humanPose_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::social_filter::humanPose_<ContainerAllocator> >::other >  humans;


  typedef boost::shared_ptr< ::social_filter::humanPoses_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::social_filter::humanPoses_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct humanPoses
typedef  ::social_filter::humanPoses_<std::allocator<void> > humanPoses;

typedef boost::shared_ptr< ::social_filter::humanPoses> humanPosesPtr;
typedef boost::shared_ptr< ::social_filter::humanPoses const> humanPosesConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::social_filter::humanPoses_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::social_filter::humanPoses_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace social_filter

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::social_filter::humanPoses_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::social_filter::humanPoses_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::social_filter::humanPoses_<ContainerAllocator> > {
  static const char* value() 
  {
    return "af15ed28aa6352fb91773931f13957ce";
  }

  static const char* value(const  ::social_filter::humanPoses_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xaf15ed28aa6352fbULL;
  static const uint64_t static_value2 = 0x91773931f13957ceULL;
};

template<class ContainerAllocator>
struct DataType< ::social_filter::humanPoses_<ContainerAllocator> > {
  static const char* value() 
  {
    return "social_filter/humanPoses";
  }

  static const char* value(const  ::social_filter::humanPoses_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::social_filter::humanPoses_<ContainerAllocator> > {
  static const char* value() 
  {
    return "humanPose[] humans\n\
================================================================================\n\
MSG: social_filter/humanPose\n\
Header header\n\
\n\
int32 id\n\
float32 x\n\
float32 y\n\
float32 theta\n\
float32 linear_velocity\n\
float32 angular_velocity\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::social_filter::humanPoses_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::social_filter::humanPoses_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.humans);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct humanPoses_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::social_filter::humanPoses_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::social_filter::humanPoses_<ContainerAllocator> & v) 
  {
    s << indent << "humans[]" << std::endl;
    for (size_t i = 0; i < v.humans.size(); ++i)
    {
      s << indent << "  humans[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::social_filter::humanPose_<ContainerAllocator> >::stream(s, indent + "    ", v.humans[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // SOCIAL_FILTER_MESSAGE_HUMANPOSES_H

