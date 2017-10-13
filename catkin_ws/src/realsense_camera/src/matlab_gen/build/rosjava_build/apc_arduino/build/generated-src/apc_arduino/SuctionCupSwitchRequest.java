package apc_arduino;

public interface SuctionCupSwitchRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/SuctionCupSwitchRequest";
  static final java.lang.String _DEFINITION = "uint8 cup_no\nbool  suction\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getCupNo();
  void setCupNo(byte value);
  boolean getSuction();
  void setSuction(boolean value);
}
