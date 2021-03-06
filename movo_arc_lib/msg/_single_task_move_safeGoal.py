# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from movo_arc_lib/single_task_move_safeGoal.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class single_task_move_safeGoal(genpy.Message):
  _md5sum = "05606b65b46e8c0338b1fb2f14bcd8c4"
  _type = "movo_arc_lib/single_task_move_safeGoal"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
#goal definition
int32 arm
float32[3] pos
float32[3] orn
float32[6] max_force
float32 time
"""
  __slots__ = ['arm','pos','orn','max_force','time']
  _slot_types = ['int32','float32[3]','float32[3]','float32[6]','float32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       arm,pos,orn,max_force,time

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(single_task_move_safeGoal, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.arm is None:
        self.arm = 0
      if self.pos is None:
        self.pos = [0.] * 3
      if self.orn is None:
        self.orn = [0.] * 3
      if self.max_force is None:
        self.max_force = [0.] * 6
      if self.time is None:
        self.time = 0.
    else:
      self.arm = 0
      self.pos = [0.] * 3
      self.orn = [0.] * 3
      self.max_force = [0.] * 6
      self.time = 0.

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
      _x = self.arm
      buff.write(_get_struct_i().pack(_x))
      buff.write(_get_struct_3f().pack(*self.pos))
      buff.write(_get_struct_3f().pack(*self.orn))
      buff.write(_get_struct_6f().pack(*self.max_force))
      _x = self.time
      buff.write(_get_struct_f().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.arm,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 12
      self.pos = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 12
      self.orn = _get_struct_3f().unpack(str[start:end])
      start = end
      end += 24
      self.max_force = _get_struct_6f().unpack(str[start:end])
      start = end
      end += 4
      (self.time,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self.arm
      buff.write(_get_struct_i().pack(_x))
      buff.write(self.pos.tostring())
      buff.write(self.orn.tostring())
      buff.write(self.max_force.tostring())
      _x = self.time
      buff.write(_get_struct_f().pack(_x))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      start = end
      end += 4
      (self.arm,) = _get_struct_i().unpack(str[start:end])
      start = end
      end += 12
      self.pos = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 12
      self.orn = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=3)
      start = end
      end += 24
      self.max_force = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=6)
      start = end
      end += 4
      (self.time,) = _get_struct_f().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
_struct_6f = None
def _get_struct_6f():
    global _struct_6f
    if _struct_6f is None:
        _struct_6f = struct.Struct("<6f")
    return _struct_6f
_struct_f = None
def _get_struct_f():
    global _struct_f
    if _struct_f is None:
        _struct_f = struct.Struct("<f")
    return _struct_f
_struct_i = None
def _get_struct_i():
    global _struct_i
    if _struct_i is None:
        _struct_i = struct.Struct("<i")
    return _struct_i
