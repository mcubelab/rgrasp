package apc_arduino;

public interface RequestServiceInfoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestServiceInfoResponse";
  static final java.lang.String _DEFINITION = "string service_md5\nstring request_md5\nstring response_md5";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getServiceMd5();
  void setServiceMd5(java.lang.String value);
  java.lang.String getRequestMd5();
  void setRequestMd5(java.lang.String value);
  java.lang.String getResponseMd5();
  void setResponseMd5(java.lang.String value);
}
