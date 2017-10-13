package realsense_camera;

public interface snapshotResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "realsense_camera/snapshotResponse";
  static final java.lang.String _DEFINITION = "# Point cloud data\nfloat32[] point_cloud_xyz # 480x640x3 (row-major order) XYZ camera coordinates, (0,0,0) are invalid points\nuint8[] point_cloud_rgb # 480x640x3 (row-major order) corresponding RGB values\n\n# Camera parameters\nfloat32[] color_camera_intrinsics # 3x3 (row-major order) color camera intrinsics\n\n# File paths\nstring[] file_paths # 3 strings with locations to camera information file (.txt), saved color image file (.png), saved depth image file (.png)";
  static final boolean _IS_SERVICE = true;
  static final boolean _IS_ACTION = false;
  float[] getPointCloudXyz();
  void setPointCloudXyz(float[] value);
  org.jboss.netty.buffer.ChannelBuffer getPointCloudRgb();
  void setPointCloudRgb(org.jboss.netty.buffer.ChannelBuffer value);
  float[] getColorCameraIntrinsics();
  void setColorCameraIntrinsics(float[] value);
  java.util.List<java.lang.String> getFilePaths();
  void setFilePaths(java.util.List<java.lang.String> value);
}
