package apc_arduino;

public interface RequestMessageInfoResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/RequestMessageInfoResponse";
  static final java.lang.String _DEFINITION = "# If found, return md5 string of system\'s version of the message, and \n# textual definition. If not found, both strings are to be empty.\nstring md5\nstring definition";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  java.lang.String getMd5();
  void setMd5(java.lang.String value);
  java.lang.String getDefinition();
  void setDefinition(java.lang.String value);
}
