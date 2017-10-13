package passive_vision;

public interface stateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "passive_vision/stateRequest";
  static final java.lang.String _DEFINITION = "# Absent Object List\nstring[] absent_objects # list of objects names that are not present in the scene\n\n# Camera Serial Number List\nstring[] camera_list # list of 1-2 camera serial numbers for which data will be agreggated\n\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.util.List<java.lang.String> getAbsentObjects();
  void setAbsentObjects(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getCameraList();
  void setCameraList(java.util.List<java.lang.String> value);
}
