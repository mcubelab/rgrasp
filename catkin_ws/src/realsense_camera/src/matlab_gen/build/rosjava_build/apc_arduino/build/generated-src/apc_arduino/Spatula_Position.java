package apc_arduino;

public interface Spatula_Position extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/Spatula_Position";
  static final java.lang.String _DEFINITION = "int32 Sp1\nint32 Sp2\nbool Sp1_moving\nbool Sp2_moving\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  int getSp1();
  void setSp1(int value);
  int getSp2();
  void setSp2(int value);
  boolean getSp1Moving();
  void setSp1Moving(boolean value);
  boolean getSp2Moving();
  void setSp2Moving(boolean value);
}
