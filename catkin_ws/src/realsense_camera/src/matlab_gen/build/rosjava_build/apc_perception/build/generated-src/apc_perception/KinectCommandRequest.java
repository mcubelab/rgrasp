package apc_perception;

public interface KinectCommandRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_perception/KinectCommandRequest";
  static final java.lang.String _DEFINITION = "int32 kinect_num\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  int getKinectNum();
  void setKinectNum(int value);
}
