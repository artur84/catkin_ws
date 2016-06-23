/* Auto-generated by genmsg_cpp for file /home/arturo/ros_works/pal/pal-ros-pkg/pal_msgs/msg/PoseWithVelocityArray.msg */
#ifndef PAL_MSGS_MESSAGE_POSEWITHVELOCITYARRAY_H
#define PAL_MSGS_MESSAGE_POSEWITHVELOCITYARRAY_H
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

#include "std_msgs/Header.h"
#include "pal_msgs/PoseWithVelocity.h"

namespace pal_msgs
{
template <class ContainerAllocator>
struct PoseWithVelocityArray_ {
  typedef PoseWithVelocityArray_<ContainerAllocator> Type;

  PoseWithVelocityArray_()
  : header()
  , poses()
  {
  }

  PoseWithVelocityArray_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , poses(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector< ::pal_msgs::PoseWithVelocity_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pal_msgs::PoseWithVelocity_<ContainerAllocator> >::other >  _poses_type;
  std::vector< ::pal_msgs::PoseWithVelocity_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pal_msgs::PoseWithVelocity_<ContainerAllocator> >::other >  poses;


  typedef boost::shared_ptr< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct PoseWithVelocityArray
typedef  ::pal_msgs::PoseWithVelocityArray_<std::allocator<void> > PoseWithVelocityArray;

typedef boost::shared_ptr< ::pal_msgs::PoseWithVelocityArray> PoseWithVelocityArrayPtr;
typedef boost::shared_ptr< ::pal_msgs::PoseWithVelocityArray const> PoseWithVelocityArrayConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace pal_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bec2ec3b87f0fff0279f4d17d9bc574d";
  }

  static const char* value(const  ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbec2ec3b87f0fff0ULL;
  static const uint64_t static_value2 = 0x279f4d17d9bc574dULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "pal_msgs/PoseWithVelocityArray";
  }

  static const char* value(const  ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
PoseWithVelocity[] poses\n\
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
================================================================================\n\
MSG: pal_msgs/PoseWithVelocity\n\
Header header\n\
geometry_msgs/Pose2D pose\n\
float64 vx\n\
float64 vy\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose2D\n\
# This expresses a position and orientation on a 2D manifold.\n\
\n\
float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const  ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.poses);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct PoseWithVelocityArray_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::pal_msgs::PoseWithVelocityArray_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "poses[]" << std::endl;
    for (size_t i = 0; i < v.poses.size(); ++i)
    {
      s << indent << "  poses[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pal_msgs::PoseWithVelocity_<ContainerAllocator> >::stream(s, indent + "    ", v.poses[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // PAL_MSGS_MESSAGE_POSEWITHVELOCITYARRAY_H
