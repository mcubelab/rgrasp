package apc_arduino;

public interface Log extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/Log";
  static final java.lang.String _DEFINITION = "\n#ROS Logging Levels\nuint8 ROSDEBUG=0\nuint8 INFO=1\nuint8 WARN=2\nuint8 ERROR=3\nuint8 FATAL=4\n\nuint8 level\nstring msg\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final byte ROSDEBUG = 0;
  static final byte INFO = 1;
  static final byte WARN = 2;
  static final byte ERROR = 3;
  static final byte FATAL = 4;
  byte getLevel();
  void setLevel(byte value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
