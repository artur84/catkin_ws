"""autogenerated by genpy from pal_msgs/PoseWithVelocityArray.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import pal_msgs.msg
import std_msgs.msg

class PoseWithVelocityArray(genpy.Message):
  _md5sum = "bec2ec3b87f0fff0279f4d17d9bc574d"
  _type = "pal_msgs/PoseWithVelocityArray"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """Header header
PoseWithVelocity[] poses

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
MSG: pal_msgs/PoseWithVelocity
Header header
geometry_msgs/Pose2D pose
float64 vx
float64 vy

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
"""
  __slots__ = ['header','poses']
  _slot_types = ['std_msgs/Header','pal_msgs/PoseWithVelocity[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,poses

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PoseWithVelocityArray, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.poses is None:
        self.poses = []
    else:
      self.header = std_msgs.msg.Header()
      self.poses = []

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
      length = len(self.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.poses:
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
        _v3 = val1.pose
        _x = _v3
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.theta))
        _x = val1
        buff.write(_struct_2d.pack(_x.vx, _x.vy))
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
      if self.poses is None:
        self.poses = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.poses = []
      for i in range(0, length):
        val1 = pal_msgs.msg.PoseWithVelocity()
        _v4 = val1.header
        start = end
        end += 4
        (_v4.seq,) = _struct_I.unpack(str[start:end])
        _v5 = _v4.stamp
        _x = _v5
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v4.frame_id = str[start:end].decode('utf-8')
        else:
          _v4.frame_id = str[start:end]
        _v6 = val1.pose
        _x = _v6
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _struct_3d.unpack(str[start:end])
        _x = val1
        start = end
        end += 16
        (_x.vx, _x.vy,) = _struct_2d.unpack(str[start:end])
        self.poses.append(val1)
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
      length = len(self.poses)
      buff.write(_struct_I.pack(length))
      for val1 in self.poses:
        _v7 = val1.header
        buff.write(_struct_I.pack(_v7.seq))
        _v8 = _v7.stamp
        _x = _v8
        buff.write(_struct_2I.pack(_x.secs, _x.nsecs))
        _x = _v7.frame_id
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _v9 = val1.pose
        _x = _v9
        buff.write(_struct_3d.pack(_x.x, _x.y, _x.theta))
        _x = val1
        buff.write(_struct_2d.pack(_x.vx, _x.vy))
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
      if self.poses is None:
        self.poses = None
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
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.poses = []
      for i in range(0, length):
        val1 = pal_msgs.msg.PoseWithVelocity()
        _v10 = val1.header
        start = end
        end += 4
        (_v10.seq,) = _struct_I.unpack(str[start:end])
        _v11 = _v10.stamp
        _x = _v11
        start = end
        end += 8
        (_x.secs, _x.nsecs,) = _struct_2I.unpack(str[start:end])
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          _v10.frame_id = str[start:end].decode('utf-8')
        else:
          _v10.frame_id = str[start:end]
        _v12 = val1.pose
        _x = _v12
        start = end
        end += 24
        (_x.x, _x.y, _x.theta,) = _struct_3d.unpack(str[start:end])
        _x = val1
        start = end
        end += 16
        (_x.vx, _x.vy,) = _struct_2d.unpack(str[start:end])
        self.poses.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2d = struct.Struct("<2d")
_struct_3I = struct.Struct("<3I")
_struct_2I = struct.Struct("<2I")
_struct_3d = struct.Struct("<3d")
