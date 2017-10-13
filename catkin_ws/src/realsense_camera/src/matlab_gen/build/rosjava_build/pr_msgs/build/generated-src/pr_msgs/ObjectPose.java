package pr_msgs;

public interface ObjectPose extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr_msgs/ObjectPose";
  static final java.lang.String _DEFINITION = "string name\n\ngeometry_msgs/Pose pose\n\nint16[] convex_hull_x\nint16[] convex_hull_y\n\nfloat32 mean_quality\nint16 used_points\n\nNameTypeValue[] properties\n\ngeometry_msgs/Pose[] pose_uncertainty_list\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.lang.String getName();
  void setName(java.lang.String value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
  short[] getConvexHullX();
  void setConvexHullX(short[] value);
  short[] getConvexHullY();
  void setConvexHullY(short[] value);
  float getMeanQuality();
  void setMeanQuality(float value);
  short getUsedPoints();
  void setUsedPoints(short value);
  java.util.List<pr_msgs.NameTypeValue> getProperties();
  void setProperties(java.util.List<pr_msgs.NameTypeValue> value);
  java.util.List<geometry_msgs.Pose> getPoseUncertaintyList();
  void setPoseUncertaintyList(java.util.List<geometry_msgs.Pose> value);
}
