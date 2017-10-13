package apc_arduino;

public interface TuneStrainGaugeRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/TuneStrainGaugeRequest";
  static final java.lang.String _DEFINITION = "uint8 sg_no\n";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  byte getSgNo();
  void setSgNo(byte value);
}
