package apc_perception;

public interface KinectCommandResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_perception/KinectCommandResponse";
  static final java.lang.String _DEFINITION = "int32 error";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  int getError();
  void setError(int value);
}
