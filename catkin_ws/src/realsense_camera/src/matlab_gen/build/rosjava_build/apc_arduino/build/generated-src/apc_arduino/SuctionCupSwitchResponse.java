package apc_arduino;

public interface SuctionCupSwitchResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/SuctionCupSwitchResponse";
  static final java.lang.String _DEFINITION = "bool success";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getSuccess();
  void setSuccess(boolean value);
}
