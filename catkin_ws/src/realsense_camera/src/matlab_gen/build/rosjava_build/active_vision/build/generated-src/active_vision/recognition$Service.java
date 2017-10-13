package active_vision;

public interface recognition$Service extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "active_vision/recognition$Service";
  static final java.lang.String _DEFINITION = "# Input parameters\nstring camera_serial_number # serial number of camera to capture images\n---\n# Zero-shot recognition results\nbool is_new_object # are we looking at a new object?\nstring[] object_name # list of object names\nfloat32[] object_confidence # list of corresponding object recognition confidence values\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
}
