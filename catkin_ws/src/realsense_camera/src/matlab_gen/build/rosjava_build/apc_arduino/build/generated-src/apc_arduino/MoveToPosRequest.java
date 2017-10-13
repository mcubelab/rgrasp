package apc_arduino;

public interface MoveToPosRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/MoveToPosRequest";
  static final java.lang.String _DEFINITION = "uint8 sp_no\nint16 pos\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getSpNo();
  void setSpNo(byte value);
  short getPos();
  void setPos(short value);
}
