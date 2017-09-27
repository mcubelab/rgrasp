#include "robot_comm.h"

RobotComm::RobotComm()
{
}

RobotComm::RobotComm(ros::NodeHandle* np, std::string id)
{
  robotname = "robot" + id;
  subscribe(np);
}

RobotComm::~RobotComm()
{
  shutdown();
}

void RobotComm::subscribe(ros::NodeHandle* np)
{
  handle_robot_Ping = 
    np->serviceClient<robot_comm::robot_Ping>(robotname + "_Ping");
  handle_robot_SetCartesian = 
    np->serviceClient<robot_comm::robot_SetCartesian>(robotname + "_SetCartesian");
  handle_robot_SetCartesianJ = 
    np->serviceClient<robot_comm::robot_SetCartesianJ>(robotname + "_SetCartesianJ");
  handle_robot_GetCartesian = 
    np->serviceClient<robot_comm::robot_GetCartesian>(robotname + "_GetCartesian");
  handle_robot_SetWorkObject = 
    np->serviceClient<robot_comm::robot_SetWorkObject>(robotname + "_SetWorkObject");
  handle_robot_SetZone = 
    np->serviceClient<robot_comm::robot_SetZone>(robotname + "_SetZone");
  handle_robot_SetMotionSupervision =
    np->serviceClient<robot_comm::robot_SetMotionSupervision>(robotname + "_SetMotionSupervision");
  handle_robot_SetTool = 
    np->serviceClient<robot_comm::robot_SetTool>(robotname + "_SetTool");
  handle_robot_SetInertia = 
    np->serviceClient<robot_comm::robot_SetInertia>(robotname + "_SetInertia");
  handle_robot_SetJoints = 
    np->serviceClient<robot_comm::robot_SetJoints>(robotname + "_SetJoints");
  handle_robot_GetJoints = 
    np->serviceClient<robot_comm::robot_GetJoints>(robotname + "_GetJoints");
  handle_robot_SetComm = 
    np->serviceClient<robot_comm::robot_SetComm>(robotname + "_SetComm");
  handle_robot_SetSpeed = 
    np->serviceClient<robot_comm::robot_SetSpeed>(robotname + "_SetSpeed");
  handle_robot_SetAcc = 
    np->serviceClient<robot_comm::robot_SetAcc>(robotname + "_SetAcc");
  handle_robot_GetState = 
    np->serviceClient<robot_comm::robot_GetState>(robotname + "_GetState");
  handle_robot_SetTrackDist = 
    np->serviceClient<robot_comm::robot_SetTrackDist>(robotname + "_SetTrackDist");
  handle_robot_Stop = 
    np->serviceClient<robot_comm::robot_Stop>(robotname + "_Stop");
  handle_robot_IsMoving = 
    np->serviceClient<robot_comm::robot_IsMoving>(robotname + "_IsMoving");
  handle_robot_SetDefaults = 
    np->serviceClient<robot_comm::robot_SetDefaults>(robotname + "_SetDefaults");
  handle_robot_GetIK = 
    np->serviceClient<robot_comm::robot_GetIK>(robotname + "_GetIK");
  handle_robot_GetFK = 
    np->serviceClient<robot_comm::robot_GetFK>(robotname + "_GetFK");
  handle_robot_Approach =
    np->serviceClient<robot_comm::robot_Approach>(robotname + "_Approach");
  handle_robot_ActivateEGM =
    np->serviceClient<robot_comm::robot_ActivateEGM>(robotname + "_ActivateEGM");
  handle_robot_IOSignal =
    np->serviceClient<robot_comm::robot_IOSignal>(robotname + "_IOSignal");
}

void RobotComm::subscribeCartesian(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const robot_comm::robot_CartesianLogConstPtr&))
{
  robot_cartesian_sub = np->subscribe(robotname + "_CartesianLog", q_len, funcPtr);  
}

void RobotComm::subscribeJoints(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const robot_comm::robot_JointsLogConstPtr&))
{
  robot_joints_sub = np->subscribe(robotname + "_JointsLog", q_len, funcPtr);  
}

void RobotComm::subscribeForce(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const robot_comm::robot_ForceLogConstPtr&))
{
  robot_force_sub = np->subscribe(robotname + "_ForceLog", q_len, funcPtr);  
}


void RobotComm::shutdown()
{
  handle_robot_Ping.shutdown();
  handle_robot_SetCartesian.shutdown();
  handle_robot_SetCartesianJ.shutdown();
  handle_robot_GetCartesian.shutdown();
  handle_robot_SetWorkObject.shutdown();
  handle_robot_SetZone.shutdown();
  handle_robot_SetMotionSupervision.shutdown();
  handle_robot_SetTool.shutdown();
  handle_robot_SetInertia.shutdown();
  handle_robot_SetJoints.shutdown();
  handle_robot_GetJoints.shutdown();
  handle_robot_SetComm.shutdown();
  handle_robot_SetSpeed.shutdown();
  handle_robot_SetAcc.shutdown();
  handle_robot_GetState.shutdown();
  handle_robot_SetTrackDist.shutdown();
  handle_robot_Stop.shutdown();
  handle_robot_IsMoving.shutdown();
  handle_robot_SetDefaults.shutdown();
  handle_robot_GetIK.shutdown();
  handle_robot_GetFK.shutdown();
  handle_robot_Approach.shutdown();
  handle_robot_ActivateEGM.shutdown();
  handle_robot_IOSignal.shutdown();
}

bool RobotComm::Ping()
{
  return handle_robot_Ping.call(robot_Ping_srv);
}
 
bool RobotComm::Approach(const geometry_msgs::Pose pose)
{
  robot_Approach_srv.request.pose = pose;
  return handle_robot_Approach.call(robot_Approach_srv);
}
 
bool RobotComm::SetCartesian(const double x, const double y, const double z, 
    const double q0, const double qx, const double qy, const double qz)
{
  robot_SetCartesian_srv.request.x = x;
	robot_SetCartesian_srv.request.y = y;
	robot_SetCartesian_srv.request.z = z;
	robot_SetCartesian_srv.request.q0 = q0;
	robot_SetCartesian_srv.request.qx = qx;
	robot_SetCartesian_srv.request.qy = qy;
	robot_SetCartesian_srv.request.qz = qz;
	return handle_robot_SetCartesian.call(robot_SetCartesian_srv);
}

bool RobotComm::SetCartesian(const double cart[7])
{
  robot_SetCartesian_srv.request.x = cart[0];
	robot_SetCartesian_srv.request.y = cart[1];
	robot_SetCartesian_srv.request.z = cart[2];
	robot_SetCartesian_srv.request.q0 = cart[3];
	robot_SetCartesian_srv.request.qx = cart[4];
	robot_SetCartesian_srv.request.qy = cart[5];
	robot_SetCartesian_srv.request.qz = cart[6];
	return handle_robot_SetCartesian.call(robot_SetCartesian_srv);
}

bool RobotComm::SetCartesian(const HomogTransf pose)
{
  Vec trans = pose.getTranslation();
  Quaternion quat = pose.getRotation().getQuaternion();
  return SetCartesian(trans[0],trans[1],trans[2],quat[0],quat[1],quat[2],quat[3]);
}

bool RobotComm::SetCartesianJ(const double x, const double y, const double z, 
    const double q0, const double qx, const double qy, const double qz)
{
  robot_SetCartesianJ_srv.request.x = x;
	robot_SetCartesianJ_srv.request.y = y;
	robot_SetCartesianJ_srv.request.z = z;
	robot_SetCartesianJ_srv.request.q0 = q0;
	robot_SetCartesianJ_srv.request.qx = qx;
	robot_SetCartesianJ_srv.request.qy = qy;
	robot_SetCartesianJ_srv.request.qz = qz;
	return handle_robot_SetCartesianJ.call(robot_SetCartesianJ_srv);
}

bool RobotComm::SetCartesianJ(const double cart[7])
{
  robot_SetCartesianJ_srv.request.x = cart[0];
	robot_SetCartesianJ_srv.request.y = cart[1];
	robot_SetCartesianJ_srv.request.z = cart[2];
	robot_SetCartesianJ_srv.request.q0 = cart[3];
	robot_SetCartesianJ_srv.request.qx = cart[4];
	robot_SetCartesianJ_srv.request.qy = cart[5];
	robot_SetCartesianJ_srv.request.qz = cart[6];
	return handle_robot_SetCartesianJ.call(robot_SetCartesianJ_srv);
}

bool RobotComm::SetCartesianJ(const HomogTransf pose)
{
  Vec trans = pose.getTranslation();
  Quaternion quat = pose.getQuaternion();
  return SetCartesianJ(trans[0],trans[1],trans[2],quat[0],quat[1],quat[2],quat[3]);
}

bool RobotComm::SetJoints(const double j[6])
{
  robot_SetJoints_srv.request.j1 = j[0];
  robot_SetJoints_srv.request.j2 = j[1];
  robot_SetJoints_srv.request.j3 = j[2];
  robot_SetJoints_srv.request.j4 = j[3];
  robot_SetJoints_srv.request.j5 = j[4];
  robot_SetJoints_srv.request.j6 = j[5];
	return handle_robot_SetJoints.call(robot_SetJoints_srv);
}

bool RobotComm::SetJoints(const double j1, const double j2, const double j3, 
    const double j4, const double j5, const double j6)
{
  robot_SetJoints_srv.request.j1 = j1;
  robot_SetJoints_srv.request.j2 = j2;
  robot_SetJoints_srv.request.j3 = j3;
  robot_SetJoints_srv.request.j4 = j4;
  robot_SetJoints_srv.request.j5 = j5;
  robot_SetJoints_srv.request.j6 = j6;
	return handle_robot_SetJoints.call(robot_SetJoints_srv);
}

bool RobotComm::SetWorkObject(const double x, const double y, const double z, 
    const double q0, const double qx, const double qy, const double qz)
{
  robot_SetWorkObject_srv.request.x = x;
	robot_SetWorkObject_srv.request.y = y;
	robot_SetWorkObject_srv.request.z = z;
	robot_SetWorkObject_srv.request.q0 = q0;
	robot_SetWorkObject_srv.request.qx = qx;
	robot_SetWorkObject_srv.request.qy = qy;
	robot_SetWorkObject_srv.request.qz = qz;
	return handle_robot_SetWorkObject.call(robot_SetWorkObject_srv);
}

bool RobotComm::SetZone(const int z)
{
  robot_SetZone_srv.request.mode = z;
  return handle_robot_SetZone.call(robot_SetZone_srv);
}

bool RobotComm::SetMotionSupervision(double sup)
{
  robot_SetMotionSupervision_srv.request.supervision = sup;
  return handle_robot_SetMotionSupervision.call(robot_SetMotionSupervision_srv);
}

bool RobotComm::SetTool(const double x, const double y, const double z, 
    const double q0, const double qx, const double qy, const double qz)
{
  robot_SetTool_srv.request.x = x;
	robot_SetTool_srv.request.y = y;
	robot_SetTool_srv.request.z = z;
  robot_SetTool_srv.request.q0 = q0;
	robot_SetTool_srv.request.qx = qx;
	robot_SetTool_srv.request.qy = qy;
	robot_SetTool_srv.request.qz = qz;
	return handle_robot_SetTool.call(robot_SetTool_srv);
}

bool RobotComm::SetInertia(const double m, const double cgx, const double cgy, 
    const double cgz, const double ix, const double iy, const double iz)
{
  robot_SetInertia_srv.request.m = m;
  robot_SetInertia_srv.request.cgx = cgx;
  robot_SetInertia_srv.request.cgy = cgy;
  robot_SetInertia_srv.request.cgz = cgz;
  robot_SetInertia_srv.request.ix = ix;
  robot_SetInertia_srv.request.iy = iy;
  robot_SetInertia_srv.request.iz = iz;
  return handle_robot_SetInertia.call(robot_SetInertia_srv);
}

bool RobotComm::SetComm(const int mode)
{
  robot_SetComm_srv.request.mode = mode;
  return handle_robot_SetComm.call(robot_SetComm_srv);
}

bool RobotComm::SetSpeed(const double tcp, const double ori)
{
  robot_SetSpeed_srv.request.tcp = tcp;
  robot_SetSpeed_srv.request.ori = ori;
  return handle_robot_SetSpeed.call(robot_SetSpeed_srv);  
}

bool RobotComm::SetAcc(const double acc, const double deacc)
{
  robot_SetAcc_srv.request.acc = acc;
  robot_SetAcc_srv.request.deacc = deacc;
  return handle_robot_SetAcc.call(robot_SetAcc_srv);  
}

bool RobotComm::IOSignal(const int output_num, const int signal)
{
  robot_IOSignal_srv.request.output_num = output_num;
  robot_IOSignal_srv.request.signal = signal;
  return handle_robot_IOSignal.call(robot_IOSignal_srv);  
}

bool RobotComm::SetTrackDist(const double pos_dist, const double ang_dist)
{
  robot_SetTrackDist_srv.request.pos_dist = pos_dist;
  robot_SetTrackDist_srv.request.ang_dist = ang_dist;
  return handle_robot_SetTrackDist.call(robot_SetTrackDist_srv);  
}

bool RobotComm::Stop()
{
  return handle_robot_Stop.call(robot_Stop_srv); 
}

bool RobotComm::IsMoving()
{
  if (handle_robot_IsMoving.call(robot_IsMoving_srv))
  {
    return robot_IsMoving_srv.response.moving;
  }
  else
    return false;
}

bool RobotComm::GetSpeed(double &tcp, double &ori)
{
  bool success = handle_robot_GetState.call(robot_GetState_srv);
  tcp = robot_GetState_srv.response.tcp;
  ori = robot_GetState_srv.response.ori;
  return success;
}

bool RobotComm::GetWorkObject(HomogTransf &workObject)
{
  double wo[7];
  bool success = GetWorkObject(wo);
  workObject = HomogTransf(wo);
  return success;
}

bool RobotComm::GetWorkObject(double workObject[7])
{
  bool success = handle_robot_GetState.call(robot_GetState_srv);
  workObject[0] = robot_GetState_srv.response.workx;
  workObject[1] = robot_GetState_srv.response.worky;
  workObject[2] = robot_GetState_srv.response.workz;
  workObject[3] = robot_GetState_srv.response.workq0;
  workObject[4] = robot_GetState_srv.response.workqx;
  workObject[5] = robot_GetState_srv.response.workqy;
  workObject[6] = robot_GetState_srv.response.workqz;
  return success;
}

bool RobotComm::GetTool(HomogTransf &tool)
{
  double t[7];
  bool success = GetTool(t);
  tool = HomogTransf(t);
  return success;
}

bool RobotComm::GetTool(double tool[7])
{
  bool success = handle_robot_GetState.call(robot_GetState_srv);
  tool[0] = robot_GetState_srv.response.toolx;
  tool[1] = robot_GetState_srv.response.tooly;
  tool[2] = robot_GetState_srv.response.toolz;
  tool[3] = robot_GetState_srv.response.toolq0;
  tool[4] = robot_GetState_srv.response.toolqx;
  tool[5] = robot_GetState_srv.response.toolqy;
  tool[6] = robot_GetState_srv.response.toolqz;
  return success; 
}

bool RobotComm::GetInertia(double inertia[7])
{
  bool success = handle_robot_GetState.call(robot_GetState_srv);
  inertia[0] = robot_GetState_srv.response.toolm;
  inertia[1] = robot_GetState_srv.response.toolcgx;
  inertia[2] = robot_GetState_srv.response.toolcgy;
  inertia[3] = robot_GetState_srv.response.toolcgz;
  inertia[4] = robot_GetState_srv.response.toolix;
  inertia[5] = robot_GetState_srv.response.tooliy;
  inertia[6] = robot_GetState_srv.response.tooliz;
  return success; 
}


bool RobotComm::GetZone(int &zone)
{
  bool success = handle_robot_GetState.call(robot_GetState_srv);
  zone = robot_GetState_srv.response.zone;
  return success;
}

bool RobotComm::GetState(double &tcp, double &ori, int &zone, HomogTransf &workObject, HomogTransf &tool, double inertia[7])
{
  double wo[7], t[7], i[7];
  bool success = GetState(tcp, ori, zone, wo, t, i);
  workObject = HomogTransf(wo);
  tool = HomogTransf(t);
  return success;
}


bool RobotComm::GetState(double &tcp, double &ori, int &zone, double workObject[7], double tool[7], double inertia[7])
{
  bool success = handle_robot_GetState.call(robot_GetState_srv);
  tcp = robot_GetState_srv.response.tcp;
  ori = robot_GetState_srv.response.ori;
  zone = robot_GetState_srv.response.zone;
  workObject[0] = robot_GetState_srv.response.workx;
  workObject[1] = robot_GetState_srv.response.worky;
  workObject[2] = robot_GetState_srv.response.workz;
  workObject[3] = robot_GetState_srv.response.workq0;
  workObject[4] = robot_GetState_srv.response.workqx;
  workObject[5] = robot_GetState_srv.response.workqy;
  workObject[6] = robot_GetState_srv.response.workqz;
  tool[0] = robot_GetState_srv.response.toolx;
  tool[1] = robot_GetState_srv.response.tooly;
  tool[2] = robot_GetState_srv.response.toolz;
  tool[3] = robot_GetState_srv.response.toolq0;
  tool[4] = robot_GetState_srv.response.toolqx;
  tool[5] = robot_GetState_srv.response.toolqy;
  tool[6] = robot_GetState_srv.response.toolqz;
  inertia[0] = robot_GetState_srv.response.toolm;
  inertia[1] = robot_GetState_srv.response.toolcgx;
  inertia[2] = robot_GetState_srv.response.toolcgy;
  inertia[3] = robot_GetState_srv.response.toolcgz;
  inertia[4] = robot_GetState_srv.response.toolix;
  inertia[5] = robot_GetState_srv.response.tooliy;
  inertia[6] = robot_GetState_srv.response.tooliz;
  return success; 
}

bool RobotComm::GetCartesian(double cart[7])
{
  double x,y,z,q0,qx,qy,qz;
  if (GetCartesian(x,y,z,q0,qx,qy,qz))
  {
    cart[0] = x;
    cart[1] = y;
    cart[2] = z;
    cart[3] = q0;
    cart[4] = qx;
    cart[5] = qy;
    cart[6] = qz;
    return true;
  }
  return false;
}

bool RobotComm::GetCartesian(double trans[3], double quat[4])
{
  double x,y,z,q0,qx,qy,qz;
  if (GetCartesian(x,y,z,q0,qx,qy,qz))
  {
    trans[0] = x;
    trans[1] = y;
    trans[2] = z;
    quat[0] = q0;
    quat[1] = qx;
    quat[2] = qy;
    quat[3] = qz;
    return true;
  }
  return false;
}

bool RobotComm::GetCartesian(Vec &trans, Quaternion &quat)
{
  double x,y,z,q0,qx,qy,qz;
  if (GetCartesian(x,y,z,q0,qx,qy,qz))
  {
    trans[0] = x;
    trans[1] = y;
    trans[2] = z;
    quat[0] = q0;
    quat[1] = qx;
    quat[2] = qy;
    quat[3] = qz;
    return true;
  }
  return false;
}


bool RobotComm::GetCartesian(HomogTransf &t)
{
  Vec trans(3);
  Quaternion quat;

  if (GetCartesian(trans, quat))
  {
    t.setTranslation(trans);
    t.setRotation(quat.getRotMat());
    return true;
  }
  return false;
}
    
bool RobotComm::GetCartesian(double &x, double &y, double &z, 
    double &q0, double &qx, double &qy, double &qz)
{
  if (handle_robot_GetCartesian.call(robot_GetCartesian_srv))
  {
    x = robot_GetCartesian_srv.response.x;
    y = robot_GetCartesian_srv.response.y;
    z = robot_GetCartesian_srv.response.z;
    q0 = robot_GetCartesian_srv.response.q0;
    qx = robot_GetCartesian_srv.response.qx;
    qy = robot_GetCartesian_srv.response.qy;
    qz = robot_GetCartesian_srv.response.qz;
    return true;
  }
  else
    return false;
}

bool RobotComm::GetJoints(double &j1, double &j2, double &j3, 
    double &j4, double &j5, double &j6)
{
  if (handle_robot_GetJoints.call(robot_GetJoints_srv))
  {
    j1 = robot_GetJoints_srv.response.j1;
    j2 = robot_GetJoints_srv.response.j2;
    j3 = robot_GetJoints_srv.response.j3;
    j4 = robot_GetJoints_srv.response.j4;
    j5 = robot_GetJoints_srv.response.j5;
    j6 = robot_GetJoints_srv.response.j6;
    return true;
  }
  else
    return false;
}

bool RobotComm::GetJoints(double j[NUM_JOINTS])
{
  if (handle_robot_GetJoints.call(robot_GetJoints_srv))
  {
    j[0] = robot_GetJoints_srv.response.j1;
    j[1] = robot_GetJoints_srv.response.j2;
    j[2] = robot_GetJoints_srv.response.j3;
    j[3] = robot_GetJoints_srv.response.j4;
    j[4] = robot_GetJoints_srv.response.j5;
    j[5] = robot_GetJoints_srv.response.j6;
    return true;
  }
  else
    return false;
}



bool RobotComm::GetIK(const HomogTransf pose, double joints[NUM_JOINTS])
{
  Vec trans = pose.getTranslation();
  Quaternion quat = pose.getRotation().getQuaternion();
  robot_GetIK_srv.request.x = trans[0];
  robot_GetIK_srv.request.y = trans[1];
  robot_GetIK_srv.request.z = trans[2];
  robot_GetIK_srv.request.q0 = quat[0];
  robot_GetIK_srv.request.qx = quat[1];
  robot_GetIK_srv.request.qy = quat[2];
  robot_GetIK_srv.request.qz = quat[3];
  if (handle_robot_GetIK.call(robot_GetIK_srv))
  {
    joints[0] = robot_GetIK_srv.response.j1;
    joints[1] = robot_GetIK_srv.response.j2;
    joints[2] = robot_GetIK_srv.response.j3;
    joints[3] = robot_GetIK_srv.response.j4;
    joints[4] = robot_GetIK_srv.response.j5;
    joints[5] = robot_GetIK_srv.response.j6;

    return true;
  }
  return false;
}

bool RobotComm::GetFK(const double joints[NUM_JOINTS], HomogTransf &pose)
{
  robot_GetFK_srv.request.j1 = joints[0];
  robot_GetFK_srv.request.j2 = joints[1];
  robot_GetFK_srv.request.j3 = joints[2];
  robot_GetFK_srv.request.j4 = joints[3];
  robot_GetFK_srv.request.j5 = joints[4];
  robot_GetFK_srv.request.j6 = joints[5];
  if (handle_robot_GetFK.call(robot_GetFK_srv))
  {
    double p[7];
    p[0] = robot_GetFK_srv.response.x;
    p[1] = robot_GetFK_srv.response.y;
    p[2] = robot_GetFK_srv.response.z;
    p[3] = robot_GetFK_srv.response.q0;
    p[4] = robot_GetFK_srv.response.qx;
    p[5] = robot_GetFK_srv.response.qy;
    p[6] = robot_GetFK_srv.response.qz;
    pose.setPose(p);

    return true;
  }
  return false;
}



bool RobotComm::moveArm(geometry_msgs::Pose pose)
{
  RobotComm::SetCartesian(double(pose.position.x), double(pose.position.y), 
      double(pose.position.z),
      pose.orientation.w, pose.orientation.x, 
      pose.orientation.y, pose.orientation.z);
  return true;
}

bool RobotComm::moveArm(double j[NUM_JOINTS])
{
  RobotComm::SetJoints(j[0],j[1],j[2],j[3],j[4],j[5]);

  return true;
}

bool RobotComm::relativeMoveArm(double x_off, double y_off, double z_off)
{
  usleep(500000);
  double x, y,z ,q0, qx, qy, qz;
  RobotComm::GetCartesian(x,y,z,q0,qx,qy,qz);
  //usleep(250000);
  ROS_INFO("Cart x:%f y:%f z:%f q0:%f qx:%f qy:%f qz:%f",x,y,z,q0,qx,qy,qz);
  RobotComm::SetCartesian(x+x_off,y+y_off,z+z_off,q0,qx,qy,qz);

  return true;
}

bool RobotComm::relativeMoveArm(double x_off, double y_off, double z_off, 
    geometry_msgs::Pose pose)
{
  RobotComm::SetCartesian(pose.position.x + x_off, 
      pose.position.y + y_off, 
      pose.position.z + z_off, 
      pose.orientation.w, 
      pose.orientation.x, 
      pose.orientation.y, 
      pose.orientation.z);

  return true;
}

bool RobotComm::invertHand(void)
{
  RobotComm::SetJoints(0.0,0.0,0.0,0.0,-90.0,0.0);

  return true;
}

bool RobotComm::moveToTop(void)
{
  RobotComm::SetCartesian(600.0,700.0,1100.0,0.707,0.0,0.0,-0.707);
  usleep(2000000);

  return true;
}

bool RobotComm::moveReset(void)
{
  RobotComm::SetJoints(0.0,0.0,0.0,0.0,0.0,0.0);

  return true;
}

bool RobotComm::setupRobot(double tcp, double ori, int zone, 
    double joints[NUM_JOINTS], geometry_msgs::Pose pose)
{
  RobotComm::SetSpeed(tcp, ori);
  RobotComm::SetZone(zone);
  RobotComm::SetJoints(joints);
  RobotComm::SetJoints(0.0, 0.0, 0.0, 0.0, 90, 0.0);
  //RobotComm::moveArm(pose);

  return true;
}


bool RobotComm::ActivateEGM()
{
  return handle_robot_ActivateEGM.call(robot_ActivateEGM_srv);
}


bool RobotComm::SetDefaults()
{
  return handle_robot_SetDefaults.call(robot_SetDefaults_srv);
}

