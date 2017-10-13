package realsense_camera;

public interface snapshotRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "realsense_camera/snapshotRequest";
  static final java.lang.String _DEFINITION = "# Input parameters\nstring camera_serial_number # serial number of camera to capture snapshot from\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getCameraSerialNumber();
  void setCameraSerialNumber(java.lang.String value);
}
