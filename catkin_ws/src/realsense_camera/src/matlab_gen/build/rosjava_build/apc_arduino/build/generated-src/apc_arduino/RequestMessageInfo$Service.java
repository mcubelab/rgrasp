package apc_arduino;

public interface RequestMessageInfo$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestMessageInfo$Service";
  static final java.lang.String _DEFINITION = "# Full message datatype, eg \"std_msgs/String\"\nstring type\n---\n# If found, return md5 string of system\'s version of the message, and \n# textual definition. If not found, both strings are to be empty.\nstring md5\nstring definition\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
