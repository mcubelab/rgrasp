package apc_arduino;

public interface LEDData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/LEDData";
  static final java.lang.String _DEFINITION = "uint16 LED0\nuint16 LED1\nuint16 LED2\nuint16 LED3\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getLED0();
  void setLED0(short value);
  short getLED1();
  void setLED1(short value);
  short getLED2();
  void setLED2(short value);
  short getLED3();
  void setLED3(short value);
}
