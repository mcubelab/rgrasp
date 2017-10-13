package pr_msgs;

public interface ObjectPoseList extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr_msgs/ObjectPoseList";
  static final java.lang.String _DEFINITION = "Header header\n\nObjectPose[] object_list\n\ntime originalTimeStamp\n\ntime requestTimeStamp\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<pr_msgs.ObjectPose> getObjectList();
  void setObjectList(java.util.List<pr_msgs.ObjectPose> value);
  org.ros.message.Time getOriginalTimeStamp();
  void setOriginalTimeStamp(org.ros.message.Time value);
  org.ros.message.Time getRequestTimeStamp();
  void setRequestTimeStamp(org.ros.message.Time value);
}
