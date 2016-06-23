"""autogenerated by genpy from trajectory_simulator/TrajectoryObservation.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import std_msgs.msg

class TrajectoryObservation(genpy.Message):
  _md5sum = "9a527b2825637f568c9382ecb8750bba"
  _type = "trajectory_simulator/TrajectoryObservation"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """uint8 FIRST = 1 #type should be 1 if this observation correspons to an starting point
uint8 LAST  = 2 #2 if it is the final point of a trajectory
Header header
uint32 object_id
uint8 type
geometry_msgs/Pose2D pose
geometry_msgs/Pose2D velocity

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
"""
  # Pseudo-constants
  FIRST = 1
  LAST = 2

  __slots__ = ['header','object_id','type','pose','velocity']
  _slot_types = ['std_msgs/Header','uint32','uint8','geometry_msgs/Pose2D','geometry_msgs/Pose2D']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,object_id,type,pose,velocity

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(TrajectoryObservation, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.object_id is None:
        self.object_id = 0
      if self.type is None:
        self.type = 0
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Pose2D()
    else:
      self.header = std_msgs.msg.Header()
      self.object_id = 0
      self.type = 0
      self.pose = geometry_msgs.msg.Pose2D()
      self.velocity = geometry_msgs.msg.Pose2D()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_IB6d.pack(_x.object_id, _x.type, _x.pose.x, _x.pose.y, _x.pose.theta, _x.velocity.x, _x.velocity.y, _x.velocity.theta))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Pose2D()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 53
      (_x.object_id, _x.type, _x.pose.x, _x.pose.y, _x.pose.theta, _x.velocity.x, _x.velocity.y, _x.velocity.theta,) = _struct_IB6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_IB6d.pack(_x.object_id, _x.type, _x.pose.x, _x.pose.y, _x.pose.theta, _x.velocity.x, _x.velocity.y, _x.velocity.theta))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose2D()
      if self.velocity is None:
        self.velocity = geometry_msgs.msg.Pose2D()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 53
      (_x.object_id, _x.type, _x.pose.x, _x.pose.y, _x.pose.theta, _x.velocity.x, _x.velocity.y, _x.velocity.theta,) = _struct_IB6d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3I = struct.Struct("<3I")
_struct_IB6d = struct.Struct("<IB6d")