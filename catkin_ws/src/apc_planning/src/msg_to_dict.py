import roslib.message
import rospy
import re
import base64
from pprint import pprint

python_to_ros_type_map = {
    'bool'    : ['bool'],
    'int'     : ['int8', 'byte', 'uint8', 'char',
                 'int16', 'uint16', 'int32', 'uint32',
                 'int64', 'uint64', 'float32', 'float64'],
    'float'   : ['float32', 'float64'],
    'str'     : ['string'],
    'unicode' : ['string'],
    'long'    : ['uint64']
}

python_primitive_types = [bool, int, long, float]
python_string_types = [str, unicode]
python_list_types = [list, tuple]

ros_time_types = ['time', 'duration']
ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                       'uint16', 'int32', 'uint32', 'int64', 'uint64',
                       'float32', 'float64', 'string']
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']
ros_binary_types_regexp = re.compile(r'(uint8|char)\[[^\]]*\]')

list_brackets = re.compile(r'\[[^\]]*\]')

def convert_ros_message_to_dictionary(message):
    """
    Takes in a ROS message and returns a Python dictionary.
    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        dict_message = convert_ros_message_to_dictionary(ros_message)
    """
    dictionary = {}
    message_fields = _get_message_fields(message)
    for field_name, field_type in message_fields:
        field_value = getattr(message, field_name)
        dictionary[field_name] = _convert_from_ros_type(field_type, field_value)

    return dictionary

def _convert_from_ros_type(field_type, field_value):
    if is_ros_binary_type(field_type, field_value):
        field_value = _convert_from_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_from_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        field_value = field_value
    elif _is_field_type_an_array(field_type):
        field_value = _convert_from_ros_array(field_type, field_value)
    else:
        field_value = convert_ros_message_to_dictionary(field_value)

    return field_value


def is_ros_binary_type(field_type, field_value):
    """ Checks if the field is a binary array one, fixed size or not
    is_ros_binary_type("uint8", 42)
    >>> False
    is_ros_binary_type("uint8[]", [42, 18])
    >>> True
    is_ros_binary_type("uint8[3]", [42, 18, 21]
    >>> True
    is_ros_binary_type("char", 42)
    >>> False
    is_ros_binary_type("char[]", [42, 18])
    >>> True
    is_ros_binary_type("char[3]", [42, 18, 21]
    >>> True
    """
    return re.search(ros_binary_types_regexp, field_type) is not None

def _convert_from_ros_binary(field_type, field_value):
    field_value = base64.standard_b64encode(field_value)
    return field_value

def _convert_from_ros_time(field_type, field_value):
    field_value = {
        'secs'  : field_value.secs,
        'nsecs' : field_value.nsecs
    }
    return field_value

def _convert_from_ros_primitive(field_type, field_value):
    return field_value

def _convert_from_ros_array(field_type, field_value):
    list_type = list_brackets.sub('', field_type)
    return [_convert_from_ros_type(list_type, value) for value in field_value]

def _get_message_fields(message):
    return zip(message.__slots__, message._slot_types)

def _is_field_type_an_array(field_type):
    return list_brackets.search(field_type) is not None
