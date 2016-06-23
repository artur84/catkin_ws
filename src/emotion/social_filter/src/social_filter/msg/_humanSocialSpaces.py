"""autogenerated by genpy from social_filter/humanSocialSpaces.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import social_filter.msg
import std_msgs.msg

class humanSocialSpaces(genpy.Message):
  _md5sum = "5f9825a90c7f72b892aaab742bafc102"
  _type = "social_filter/humanSocialSpaces"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """humanSocialSpace[] socialSpaces

================================================================================
MSG: social_filter/humanSocialSpace
Header header

int32 id
int32 human_id
float32 size
float32 sigma_h
float32 sigma_r
float32 sigma_s
int32   attractiveness

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
  __slots__ = ['socialSpaces']
  _slot_types = ['social_filter/humanSocialSpace[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       socialSpaces

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(humanSocialSpaces, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.socialSpaces is None:
        self.socialSpaces = []
    else:
      self.socialSpaces = []

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
      length = len(self.socialSpaces)
      buff.write(_struct_I.pack(length))
      for val1 in self.socialSpaces:
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
        buff.write(_struct_2i4fi.pack(_x.id, _x.human_id, _x.size, _x.sigma_h, _x.sigma_r, _x.sigma_s, _x.attractiveness))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.socialSpaces is None:
        self.socialSpaces = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.socialSpaces = []
      for i in range(0, length):
        val1 = social_filter.msg.humanSocialSpace()
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
        end += 28
        (_x.id, _x.human_id, _x.size, _x.sigma_h, _x.sigma_r, _x.sigma_s, _x.attractiveness,) = _struct_2i4fi.unpack(str[start:end])
        self.socialSpaces.append(val1)
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
      length = len(self.socialSpaces)
      buff.write(_struct_I.pack(length))
      for val1 in self.socialSpaces:
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
        buff.write(_struct_2i4fi.pack(_x.id, _x.human_id, _x.size, _x.sigma_h, _x.sigma_r, _x.sigma_s, _x.attractiveness))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.socialSpaces is None:
        self.socialSpaces = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.socialSpaces = []
      for i in range(0, length):
        val1 = social_filter.msg.humanSocialSpace()
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
        end += 28
        (_x.id, _x.human_id, _x.size, _x.sigma_h, _x.sigma_r, _x.sigma_s, _x.attractiveness,) = _struct_2i4fi.unpack(str[start:end])
        self.socialSpaces.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_2i4fi = struct.Struct("<2i4fi")
_struct_2I = struct.Struct("<2I")