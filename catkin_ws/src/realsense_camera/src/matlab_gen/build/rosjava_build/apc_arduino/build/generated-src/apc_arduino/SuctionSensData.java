package apc_arduino;

public interface SuctionSensData extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/SuctionSensData";
  static final java.lang.String _DEFINITION = "uint16 SucSen0\nuint16 SucSen1\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  short getSucSen0();
  void setSucSen0(short value);
  short getSucSen1();
  void setSucSen1(short value);
}
