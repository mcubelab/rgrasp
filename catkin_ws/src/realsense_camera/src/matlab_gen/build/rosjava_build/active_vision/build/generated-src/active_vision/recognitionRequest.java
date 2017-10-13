package active_vision;

public interface recognitionRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "active_vision/recognitionRequest";
  static final java.lang.String _DEFINITION = "# Input parameters\nstring camera_serial_number # serial number of camera to capture images\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getCameraSerialNumber();
  void setCameraSerialNumber(java.lang.String value);
}
