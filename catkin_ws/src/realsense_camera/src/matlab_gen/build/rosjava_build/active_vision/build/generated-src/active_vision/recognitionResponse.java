package active_vision;

public interface recognitionResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "active_vision/recognitionResponse";
  static final java.lang.String _DEFINITION = "# Zero-shot recognition results\nbool is_new_object # are we looking at a new object?\nstring[] object_name # list of object names\nfloat32[] object_confidence # list of corresponding object recognition confidence values";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  boolean getIsNewObject();
  void setIsNewObject(boolean value);
  java.util.List<java.lang.String> getObjectName();
  void setObjectName(java.util.List<java.lang.String> value);
  float[] getObjectConfidence();
  void setObjectConfidence(float[] value);
}
