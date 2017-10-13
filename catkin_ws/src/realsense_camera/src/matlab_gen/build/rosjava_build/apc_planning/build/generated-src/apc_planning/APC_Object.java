package apc_planning;

public interface APC_Object extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_planning/APC_Object";
  static final java.lang.String _DEFINITION = "string ObjId\nint64 binId\nfloat64 score\nfloat64[] position\n\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  java.lang.String getObjId();
  void setObjId(java.lang.String value);
  long getBinId();
  void setBinId(long value);
  double getScore();
  void setScore(double value);
  double[] getPosition();
  void setPosition(double[] value);
}
