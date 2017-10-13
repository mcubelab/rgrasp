package apc_planning;

public interface APC_Scenario extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_planning/APC_Scenario";
  static final java.lang.String _DEFINITION = "Header header\nint32 num_objects\nAPC_Object[] objects\nint32 goal_object\nstring primitive\nint32 result\n#sensor_msgs/PointCloud2 point_cloud\n#sensor_msgs/Image image\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getNumObjects();
  void setNumObjects(int value);
  java.util.List<apc_planning.APC_Object> getObjects();
  void setObjects(java.util.List<apc_planning.APC_Object> value);
  int getGoalObject();
  void setGoalObject(int value);
  java.lang.String getPrimitive();
  void setPrimitive(java.lang.String value);
  int getResult();
  void setResult(int value);
}
