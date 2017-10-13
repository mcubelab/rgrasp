package apc_arduino;

public interface Adc extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/Adc";
  static final java.lang.String _DEFINITION = "uint16 adc0\nuint16 adc1\nuint16 adc2\nuint16 adc3\nuint16 adc4\nuint16 adc5\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getAdc0();
  void setAdc0(short value);
  short getAdc1();
  void setAdc1(short value);
  short getAdc2();
  void setAdc2(short value);
  short getAdc3();
  void setAdc3(short value);
  short getAdc4();
  void setAdc4(short value);
  short getAdc5();
  void setAdc5(short value);
}
