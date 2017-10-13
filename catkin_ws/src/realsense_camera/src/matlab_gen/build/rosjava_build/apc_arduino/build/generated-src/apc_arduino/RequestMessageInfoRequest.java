package apc_arduino;

public interface RequestMessageInfoRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestMessageInfoRequest";
  static final java.lang.String _DEFINITION = "# Full message datatype, eg \"std_msgs/String\"\nstring type\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getType();
  void setType(java.lang.String value);
}
