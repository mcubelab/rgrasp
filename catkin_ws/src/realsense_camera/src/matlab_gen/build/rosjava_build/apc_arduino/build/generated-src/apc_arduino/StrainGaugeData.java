package apc_arduino;

public interface StrainGaugeData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/StrainGaugeData";
  static final java.lang.String _DEFINITION = "int16 strain0\nuint8 level0\nint16 strain1\nuint8 level1\nint16 strain2\nuint8 level2\nint16 strain3\nuint8 level3\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getStrain0();
  void setStrain0(short value);
  byte getLevel0();
  void setLevel0(byte value);
  short getStrain1();
  void setStrain1(short value);
  byte getLevel1();
  void setLevel1(byte value);
  short getStrain2();
  void setStrain2(short value);
  byte getLevel2();
  void setLevel2(byte value);
  short getStrain3();
  void setStrain3(short value);
  byte getLevel3();
  void setLevel3(byte value);
}
