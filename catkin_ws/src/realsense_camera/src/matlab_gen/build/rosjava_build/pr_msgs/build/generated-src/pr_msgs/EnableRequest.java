package pr_msgs;

public interface EnableRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr_msgs/EnableRequest";
  static final java.lang.String _DEFINITION = "bool Enable\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getEnable();
  void setEnable(boolean value);
}
