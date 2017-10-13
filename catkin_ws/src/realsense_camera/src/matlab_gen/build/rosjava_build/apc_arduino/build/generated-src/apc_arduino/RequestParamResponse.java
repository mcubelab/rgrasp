package apc_arduino;

public interface RequestParamResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestParamResponse";
  static final java.lang.String _DEFINITION = "\nint32[]   ints\nfloat32[] floats\nstring[]  strings";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  int[] getInts();
  void setInts(int[] value);
  float[] getFloats();
  void setFloats(float[] value);
  java.util.List<java.lang.String> getStrings();
  void setStrings(java.util.List<java.lang.String> value);
}
