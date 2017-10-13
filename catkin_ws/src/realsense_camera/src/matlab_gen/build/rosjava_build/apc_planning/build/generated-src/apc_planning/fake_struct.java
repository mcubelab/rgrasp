package apc_planning;

public interface fake_struct extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_planning/fake_struct";
  static final java.lang.String _DEFINITION = "float64 x\nfloat64 y";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
}
