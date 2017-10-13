package pr_msgs;

public interface EnableResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr_msgs/EnableResponse";
  static final java.lang.String _DEFINITION = "bool ok \nstring reason";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getOk();
  void setOk(boolean value);
  java.lang.String getReason();
  void setReason(java.lang.String value);
}
