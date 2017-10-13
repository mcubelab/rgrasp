package apc_arduino;

public interface HallEffectData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/HallEffectData";
  static final java.lang.String _DEFINITION = "uint16 HE0\nuint16 HE1\nuint16 HE2\nuint16 HE3\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getHE0();
  void setHE0(short value);
  short getHE1();
  void setHE1(short value);
  short getHE2();
  void setHE2(short value);
  short getHE3();
  void setHE3(short value);
}
