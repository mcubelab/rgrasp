package manual_fit;

public interface GetPoseRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "manual_fit/GetPoseRequest";
  static final java.lang.String _DEFINITION = "string obj_id\nstring bin_id\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getObjId();
  void setObjId(java.lang.String value);
  java.lang.String getBinId();
  void setBinId(java.lang.String value);
}
