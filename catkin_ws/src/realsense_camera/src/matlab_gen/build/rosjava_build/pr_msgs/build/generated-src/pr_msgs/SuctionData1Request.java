package pr_msgs;

public interface SuctionData1Request extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr_msgs/SuctionData1Request";
  static final java.lang.String _DEFINITION = "string a\nstring b\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getA();
  void setA(java.lang.String value);
  java.lang.String getB();
  void setB(java.lang.String value);
}
