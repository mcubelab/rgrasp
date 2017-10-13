package apc_arduino;

public interface RequestParamRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestParamRequest";
  static final java.lang.String _DEFINITION = "string name\n\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getName();
  void setName(java.lang.String value);
}
