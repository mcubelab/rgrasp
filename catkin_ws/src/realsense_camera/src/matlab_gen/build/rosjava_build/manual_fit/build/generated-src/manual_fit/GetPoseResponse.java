package manual_fit;

public interface GetPoseResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "manual_fit/GetPoseResponse";
  static final java.lang.String _DEFINITION = "pr_msgs/ObjectPoseList pa";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  pr_msgs.ObjectPoseList getPa();
  void setPa(pr_msgs.ObjectPoseList value);
}
