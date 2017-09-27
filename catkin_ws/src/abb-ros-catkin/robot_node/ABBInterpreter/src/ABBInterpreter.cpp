#include "ABBInterpreter.h"
#include <sstream>
#include <iomanip>
#include <iostream>

using namespace std;

// begin helper section
char buff[1024];
string stringFromInstructionCode(int instructionCode, int idCode){
  sprintf(buff,"%.2d %.3d #",instructionCode, idCode); 
  return string(buff);
}

string stringFromInstructionCodeNoEnding(int instructionCode, int idCode){
  sprintf(buff,"%.2d %.3d ",instructionCode, idCode); 
  return string(buff);
}

// end helper section
/**
  * Formats message to ping the ABB robot.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */

string ABBInterpreter::pingRobot(int idCode)
{
  return stringFromInstructionCode(0,idCode);
}

/**
  * Formats message to set the cartesian coordinates of the ABB robot.
  * The coordinates are always with respect to the currently defined work object and tool.
  * @param x X-coordinate of the robot.
  * @param y Y-coordinate of the robot.
  * @param z Z-coordinate of the robot.
  * @param q0 First component of the orientation quaternion.
  * @param qx Second component of the orientation quaternion.
  * @param qy Third component of the orientation quaternion.
  * @param qz Fourth component of the orientation quaternion.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(1,idCode);
  sprintf(buff,"%+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf ",x,y,z,q0,qx,qy,qz);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to set the cartesian coordinates of the ABB robot, but
  * using a joint move to get there.
  * The coordinates are always with respect to the currently defined work object and tool.
  * @param x X-coordinate of the robot.
  * @param y Y-coordinate of the robot.
  * @param z Z-coordinate of the robot.
  * @param q0 First component of the orientation quaternion.
  * @param qx Second component of the orientation quaternion.
  * @param qy Third component of the orientation quaternion.
  * @param qz Fourth component of the orientation quaternion.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setCartesianJ(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(1,idCode);
  sprintf(buff,"%+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf ",x,y,z,q0,qx,qy,qz);  msg += buff ;
  msg += "j #";
  return (msg);
}

/**
  * Formats message to set the joint coordinates of the ABB robot.
  * @param joint1 Value of joint 1.
  * @param joint2 Value of joint 2.
  * @param joint3 Value of joint 3.
  * @param joint4 Value of joint 4.
  * @param joint5 Value of joint 5.
  * @param joint6 Value of joint 6.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(2,idCode);
  sprintf(buff,"%+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf ",joint1,joint2,joint3,joint4,joint5,joint6);  msg += buff ;
  return (msg+"#");
}

string ABBInterpreter::getIK(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(12,idCode);
  sprintf(buff,"%+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf ",x,y,z,q0,qx,qy,qz);  msg += buff ;
  return (msg+"#");
}

string ABBInterpreter::getFK(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(13,idCode);
  sprintf(buff,"%+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf ",joint1,joint2,joint3,joint4,joint5,joint6);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to query the ABB robot for its cartesian coordinates.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::getCartesian(int idCode)
{
  return stringFromInstructionCode(3,idCode);
}

/**
  * Formats message to query the ABB robot for its joint axis coordinates.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::getJoints(int idCode)
{
  return stringFromInstructionCode(4,idCode);
}

  /**
  * Formats message to set the motion suppervision thershold of the ABB robot.
  * @param supervision Percentage of Motion supervision ([0,300]).
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setMotionSupervision(double sup, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(5,idCode);
  sprintf(buff,"%08.1lf ",sup);  msg += buff ;
  return (msg + "#");
}

/**
  * Formats message to define the tool coordinates.
  * @param x X-coordinate of the tool.
  * @param y Y-coordinate of the tool.
  * @param z Z-coordinate of the tool.
  * @param q0 First component of the orientation quaternion of the tool.
  * @param qx Second component of the orientation quaternion of the tool.
  * @param qy Third component of the orientation quaternion of the tool.
  * @param qz Fourth component of the orientation quaternion of the tool.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setTool(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(6,idCode);
  sprintf(buff,"%+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf ",x,y,z,q0,qx,qy,qz);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to define the tool coordinates.
  * @param m Mass of the tool in Kg.
  * @param cgx X-coordinate of the CG of the tool in mm.
  * @param cgy Y-coordinate of the CG of the tool in mm.
  * @param cgz Z-coordinate of the CG of the tool in mm.
  * @param ix Moment of inertia with respect to axis X.
  * @param iy Moment of inertia with respect to axis Y.
  * @param iz Moment of inertia with respect to axis Z.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setInertia(double m, double cgx, double cgy, double cgz, double ix, double iy, double iz, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(14,idCode);
  sprintf(buff,"%+08.5lf %+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf ",m,cgx,cgy,cgz,ix,iy,iz);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to define the coordinates of the work object reference frame.
  * @param x X-coordinate of the work object reference frame.
  * @param y Y-coordinate of the work object reference frame.
  * @param z Z-coordinate of the work object reference frame.
  * @param q0 First component of the orientation quaternion of the work object reference frame.
  * @param qx Second component of the orientation quaternion of the work object reference frame.
  * @param qy Third component of the orientation quaternion of the work object reference frame.
  * @param qz Fourth component of the orientation quaternion of the work object reference frame.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(7,idCode);
  sprintf(buff,"%+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf ",x,y,z,q0,qx,qy,qz);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to set the speed of the ABB robot.
  * The values specified for tcp and ori are modified by the percentage of override specified by the operator in the teach pendant. 
  * @param tcp Linear speed of the TCP in mm/s (max recommended value ~ 500).
  * @param ori Reorientation speed of the TCP in deg/s (max recommended value ~ 150).
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setSpeed(double tcp, double ori, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(8,idCode);
  sprintf(buff,"%08.1lf %08.2lf ",tcp, ori);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to set the TCP acceleration of the ABB robot.
  * @param acc Linear acceleration of the TCP in mm/s^2 (0.1-28 m/s^2).
  * @param deacc Linear deacceleration of the TCP in mm/s^2 (0.1-28 m/s^2).
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setAcc(double acc, double deacc, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(15,idCode);
  sprintf(buff,"%08.2lf %08.2lf ",acc, deacc);  msg += buff ;
  return (msg+"#");
}

/**
  * Formats message to set the zone mode of the ABB robot (distance from where to interpolate to the next destination).
  * @param fine Motion mode: 1 - Stop point. 0 - Fly by point.
  * @param tcp_mm linear distance from target point to begin to interpolate the position of the TCP (recommended = 5.0mm)
  * @param ori_mm linear distance from the target point to begin to interpolate the orientation of the TCP (recommended = 5.0mm)
  * @param ori_deg angular distance from the target point to begin to interpolate the orientation of the TCP (recommended = 1.0deg)
                   Hi.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setZone(bool fine, double tcp_mm, double ori_mm, double ori_deg, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(9,idCode);
  sprintf(buff,"%.1d %.2lf %.2lf %.2lf ",fine,tcp_mm,ori_mm,ori_deg);  msg += buff ;
  return (msg+"#");
}

string ABBInterpreter::addBuffer(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  //appends single target to the buffer
  // move will execute at current speed (which you can change between addBuffer calls)
  
  string msg = stringFromInstructionCodeNoEnding(30,idCode);
  sprintf(buff,"%+08.1lf %+08.1lf %+08.1lf %+08.5lf %+08.5lf %+08.5lf %+08.5lf ",x,y,z,q0,qx,qy,qz);  msg += buff ;
  return (msg+"#");
}

string ABBInterpreter::clearBuffer(int idCode)
{
  return stringFromInstructionCode(31,idCode);
}

string ABBInterpreter::lenBuffer(int idCode)
{
  return stringFromInstructionCode(32,idCode);
}

string ABBInterpreter::executeBuffer(int idCode)
{
  return stringFromInstructionCode(33,idCode);
}

string ABBInterpreter::addJointPosBuffer(double q1, double q2, double q3, double q4, double q5, double q6, int idCode)
{
  string msg = stringFromInstructionCodeNoEnding(37,idCode);
  sprintf(buff,"%+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf %+08.2lf ",q1,q2,q3,q4,q5,q6);  msg += buff ;
  return (msg+"#");
}
        
string ABBInterpreter::clearJointPosBuffer(int idCode)
{
  return stringFromInstructionCode(38,idCode);
}

string ABBInterpreter::executeJointPosBuffer(int idCode)
{
  return stringFromInstructionCode(40,idCode);
}  

string ABBInterpreter::connectRRI(int idCode)
{
  return stringFromInstructionCode(50,idCode);
}

string ABBInterpreter::closeRRI(int idCode)
{
  return stringFromInstructionCode(51,idCode);
}

/**
  * Formats message to define the coordinates of the work object reference frame.
  * @param refFrame The coordinate system the soft direction is related to.
  *                 CSS_REFFRAME_TOOL: 1 Softness direction will be in relation to current tool.
                    CSS_REFFRAME_WOBJ: 2 Softness direction will be in relation to current work object.
  * @param refOrient This argument gives the possibility to rotate the coordinate system described by RefFrame.
  * @param softDir  The Cartesian direction in which the robot will be soft. Soft direction is in relation
  *                 to RefFrame.
  *                 CSS_X := 1;
                    CSS_Y := 2;
                    CSS_Z := 3;
                    CSS_XY := 4;
                    CSS_XZ := 5;
                    CSS_YZ := 6;
                    CSS_XYZ := 7;
                    CSS_XYRZ := 8;
  * @param stiffness This argument describes how strongly the robot tries to move back to the reference
  *                  point when it is pushed away from that point. It is a percentage of a configured
  *                  value where 0 gives no spring effect of going back to the reference point.
  * @param stiffnessNonSoftDir This argument sets the softness for all directions that are not defined as soft by the argument SoftDir.
  * @param allowMove When this switch is used movement instructions will be allowed during the activated 
  *                  soft mode. Note that using \AllowMove will internally increase the value of the 
  *                  stiffness parameter.
  * @param ramp This argument defines how fast the softness is implemented, as a percentage of 
  *             the value set by the system parameter Activation smoothness time. Can be set to
  *             between 1 and 500%.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
  
string ABBInterpreter::actCSS(int refFrame, double refOrient_q0, double refOrient_qx, double refOrient_qy, double refOrient_qz, 
                              int softDir, double stiffness, double stiffnessNonSoftDir, int allowMove, double ramp,
                              int idCode)
{
  stringstream ss;
  ss << "60 " //instruction code;
     << idCode << " "
     << setprecision(13) << refFrame << " " 
     << refOrient_q0 << " " << refOrient_qx << " " << refOrient_qy << " " << refOrient_qz << " "
     << softDir << " " << stiffness << " " << stiffnessNonSoftDir << " " << allowMove << " " 
     << ramp << " #";

  return ss.str();
}

/**
  * Formats message to set the cartesian coordinates of the ABB robot during deactCSS.
  * The coordinates are always with respect to the currently defined work object and tool.
  * @param x X-coordinate of the robot.
  * @param y Y-coordinate of the robot.
  * @param z Z-coordinate of the robot.
  * @param q0 First component of the orientation quaternion.
  * @param qx Second component of the orientation quaternion.
  * @param qy Third component of the orientation quaternion.
  * @param qz Fourth component of the orientation quaternion.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
  
string ABBInterpreter::deactCSS(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  stringstream ss;
  ss << "61 " //instruction code;
     << idCode << " "
     << setprecision(13) << x << " " << y << " " << z << " " << q0 << " " << qx << " " << qy << " " << qz << " #";

  return ss.str();
}

string ABBInterpreter::actEGM(int idCode)
{
  return stringFromInstructionCode(70,idCode);
}

// outputNum: 1, 2, 3, 4
// iosignal: 0, 1
string ABBInterpreter::iosignal(int outputNum, int signal, int idCode)
{
  stringstream ss;
  ss << "10 " //instruction code;
     << idCode << " " << outputNum << " " << signal << " #";

  return ss.str();
}


/**
  * Formats message to close the connection with the server in the ABB robot.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::closeConnection(int idCode)
{
  return stringFromInstructionCode(99,idCode);
}

/**
  * Parser for the answer from the controller to the command getCartesian().
  * @param msg String message to parse.
  * @param x Placer for the X-coordinate of the ABB robot.
  * @param y Placer for the Y-coordinate of the ABB robot.
  * @param z Placer for the Z-coordinate of the ABB robot.
  * @param q0 Placer for the first component of the orientation quaternion of the ABB robot.
  * @param qx Placer for the second component of the orientation quaternion of the ABB robot.
  * @param qy Placer for the third component of the orientation quaternion of the ABB robot.
  * @param qz Placer for the fourth component of the orientation quaternion of the ABB robot.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int ABBInterpreter::parseCartesian(std::string msg, double *x, double *y, 
    double *z,double *q0, double *qx, double *qy, double*qz)
{
  int ok, idCode;
  sscanf(msg.c_str(),"%*d %d %d %*f %lf %lf %lf %lf %lf %lf %lf",&idCode,&ok,x,y,z,q0,qx,qy,qz);
  if (ok)
    return idCode;
  else
    return -1;
}

/**
  * Parser for the answer from the controller to the command getJoints().
  * @param msg String message to parse.
  * @param joint1 Placer for the joint 1 of the ABB robot.
  * @param joint2 Placer for the joint 2 of the ABB robot.
  * @param joint3 Placer for the joint 3 of the ABB robot.
  * @param joint4 Placer for the joint 4 of the ABB robot.
  * @param joint5 Placer for the joint 5 of the ABB robot.
  * @param joint6 Placer for the joint 6 of the ABB robot.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int ABBInterpreter::parseJoints(std::string msg,  double *joint1, 
    double *joint2, double *joint3, double *joint4, 
    double *joint5, double *joint6)
{
  int ok, idCode;
  sscanf(msg.c_str(),"%*d %d %d %*f %lf %lf %lf %lf %lf %lf",&idCode,&ok,joint1,joint2,joint3,joint4,joint5,joint6);
  if (ok)
    return idCode;
  else
    return -1;
}
