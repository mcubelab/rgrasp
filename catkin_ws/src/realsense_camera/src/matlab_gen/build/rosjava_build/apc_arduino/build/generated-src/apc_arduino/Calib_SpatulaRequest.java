package apc_arduino;

public interface Calib_SpatulaRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/Calib_SpatulaRequest";
  static final java.lang.String _DEFINITION = "uint8 sp_no\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getSpNo();
  void setSpNo(byte value);
}
