"""autogenerated by genpy from social_filter/humanPoses.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import social_filter.msg
import std_msgs.msg

class humanPoses(genpy.Message):
  _md5sum = "af15ed28aa6352fb91773931f13957ce"
  _type = "social_filter/humanPoses"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """humanPose[] humans
================================================================================
MSG: social_filter/humanPose
Header header

int32 id
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity

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

"""
  __slots__ = ['humans']
  _slot_types = ['social_filter/humanPose[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       humans

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(humanPoses, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.humans is None:
        self.humans = []
    else:
      self.humans = []

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
      length = len(self.humans)
      buff.write(_struct_I.pack(length))
      for val1 in self.humans:
        _v1 = val1.header
        buff.write(_struct_I.pack(_v1.seq))
        _v2 = _v1.stamp
        _x = _v2
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v1.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_i5f.pack(_x.id, _x.x, _x.y, _x.theta, _x.linear_velocity, _x.angular_velocity))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.humans is None:
        self.humans = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.humans = []
      for i in range(0, length):
        val1 = social_filter.msg.humanPose()
        _v3 = val1.header
        start = end
        end += 4
        (_v3.seq,) = _struct_I.unpack(str[start:end])
        _v4 = _v3.stamp
        _x = _v4
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v3.frame_id = str[start:end].decode('utf-8')
        else:
          _v3.frame_id = str[start:end]
        _x = val1
        start = end
        end += 24
        (_x.id, _x.x, _x.y, _x.theta, _x.linear_velocity, _x.angular_velocity,) = _struct_i5f.unpack(str[start:end])
        self.humans.append(val1)
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
      length = len(self.humans)
      buff.write(_struct_I.pack(length))
      for val1 in self.humans:
        _v5 = val1.header
        buff.write(_struct_I.pack(_v5.seq))
        _v6 = _v5.stamp
        _x = _v6
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v5.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_i5f.pack(_x.id, _x.x, _x.y, _x.theta, _x.linear_velocity, _x.angular_velocity))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.humans is None:
        self.humans = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.humans = []
      for i in range(0, length):
        val1 = social_filter.msg.humanPose()
        _v7 = val1.header
        start = end
        end += 4
        (_v7.seq,) = _struct_I.unpack(str[start:end])
        _v8 = _v7.stamp
        _x = _v8
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v7.frame_id = str[start:end].decode('utf-8')
        else:
          _v7.frame_id = str[start:end]
        _x = val1
        start = end
        end += 24
        (_x.id, _x.x, _x.y, _x.theta, _x.linear_velocity, _x.angular_velocity,) = _struct_i5f.unpack(str[start:end])
        self.humans.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2I = struct.Struct("<2I")
_struct_i5f = struct.Struct("<i5f")
