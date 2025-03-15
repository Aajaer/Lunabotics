# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rosidl_typesupport_microxrcedds_test_msg:msg/Regression10.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Regression10(type):
    """Metaclass of message 'Regression10'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rosidl_typesupport_microxrcedds_test_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rosidl_typesupport_microxrcedds_test_msg.msg.Regression10')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__regression10
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__regression10
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__regression10
            cls._TYPE_SUPPORT = module.type_support_msg__msg__regression10
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__regression10

            from rosidl_typesupport_microxrcedds_test_msg.msg import Regression11
            if Regression11.__class__._TYPE_SUPPORT is None:
                Regression11.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Regression10(metaclass=Metaclass_Regression10):
    """Message class 'Regression10'."""

    __slots__ = [
        '_a',
        '_b',
        '_c',
        '_d',
        '_e',
    ]

    _fields_and_field_types = {
        'a': 'octet',
        'b': 'uint16',
        'c': 'uint16',
        'd': 'uint8',
        'e': 'rosidl_typesupport_microxrcedds_test_msg/Regression11[10]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('octet'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['rosidl_typesupport_microxrcedds_test_msg', 'msg'], 'Regression11'), 10),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.a = kwargs.get('a', bytes([0]))
        self.b = kwargs.get('b', int())
        self.c = kwargs.get('c', int())
        self.d = kwargs.get('d', int())
        from rosidl_typesupport_microxrcedds_test_msg.msg import Regression11
        self.e = kwargs.get(
            'e',
            [Regression11() for x in range(10)]
        )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.a != other.a:
            return False
        if self.b != other.b:
            return False
        if self.c != other.c:
            return False
        if self.d != other.d:
            return False
        if self.e != other.e:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def a(self):
        """Message field 'a'."""
        return self._a

    @a.setter
    def a(self, value):
        if __debug__:
            from collections.abc import ByteString
            assert \
                (isinstance(value, (bytes, ByteString)) and
                 len(value) == 1), \
                "The 'a' field must be of type 'bytes' or 'ByteString' with length 1"
        self._a = value

    @builtins.property
    def b(self):
        """Message field 'b'."""
        return self._b

    @b.setter
    def b(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'b' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'b' field must be an unsigned integer in [0, 65535]"
        self._b = value

    @builtins.property
    def c(self):
        """Message field 'c'."""
        return self._c

    @c.setter
    def c(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'c' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'c' field must be an unsigned integer in [0, 65535]"
        self._c = value

    @builtins.property
    def d(self):
        """Message field 'd'."""
        return self._d

    @d.setter
    def d(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'd' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'd' field must be an unsigned integer in [0, 255]"
        self._d = value

    @builtins.property
    def e(self):
        """Message field 'e'."""
        return self._e

    @e.setter
    def e(self, value):
        if __debug__:
            from rosidl_typesupport_microxrcedds_test_msg.msg import Regression11
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 10 and
                 all(isinstance(v, Regression11) for v in value) and
                 True), \
                "The 'e' field must be a set or sequence with length 10 and each value of type 'Regression11'"
        self._e = value
