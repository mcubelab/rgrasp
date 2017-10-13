package apc_arduino;

public interface ServiceBridgeRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/ServiceBridgeRequest";
  static final java.lang.String _DEFINITION = "uint8 service\nuint8 channel\nint16 value\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getService();
  void setService(byte value);
  byte getChannel();
  void setChannel(byte value);
  short getValue();
  void setValue(short value);
}
