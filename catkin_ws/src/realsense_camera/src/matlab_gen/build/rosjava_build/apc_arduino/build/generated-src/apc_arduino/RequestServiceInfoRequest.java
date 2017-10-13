package apc_arduino;

public interface RequestServiceInfoRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestServiceInfoRequest";
  static final java.lang.String _DEFINITION = "# service name\nstring service\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getService();
  void setService(java.lang.String value);
}
