package apc_arduino;

public interface SucCupOri extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/SucCupOri";
  static final java.lang.String _DEFINITION = "int8 sc1\nint8 sc2\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  byte getSc1();
  void setSc1(byte value);
  byte getSc2();
  void setSc2(byte value);
}
