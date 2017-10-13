package apc_arduino;

public interface TopicInfo extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "apc_arduino/TopicInfo";
  static final java.lang.String _DEFINITION = "# special topic_ids\nuint16 ID_PUBLISHER=0\nuint16 ID_SUBSCRIBER=1\nuint16 ID_SERVICE_SERVER=2\nuint16 ID_SERVICE_CLIENT=4\nuint16 ID_PARAMETER_REQUEST=6\nuint16 ID_LOG=7\nuint16 ID_TIME=10\nuint16 ID_TX_STOP=11\n\n# The endpoint ID for this topic\nuint16 topic_id\n\nstring topic_name\nstring message_type\n\n# MD5 checksum for this message type\nstring md5sum\n\n# size of the buffer message must fit in\nint32 buffer_size\n";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  static final short ID_PUBLISHER = 0;
  static final short ID_SUBSCRIBER = 1;
  static final short ID_SERVICE_SERVER = 2;
  static final short ID_SERVICE_CLIENT = 4;
  static final short ID_PARAMETER_REQUEST = 6;
  static final short ID_LOG = 7;
  static final short ID_TIME = 10;
  static final short ID_TX_STOP = 11;
  short getTopicId();
  void setTopicId(short value);
  java.lang.String getTopicName();
  void setTopicName(java.lang.String value);
  java.lang.String getMessageType();
  void setMessageType(java.lang.String value);
  java.lang.String getMd5sum();
  void setMd5sum(java.lang.String value);
  int getBufferSize();
  void setBufferSize(int value);
}
