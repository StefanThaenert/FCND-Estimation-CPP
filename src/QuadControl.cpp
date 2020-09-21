#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  //cmd.desiredThrustsN[0] = mass * 9.81f / 4.f; // front left
  //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
  //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
  //cmd.desiredThrustsN[3] = mass * 9.81f / 4.f; // rear right

  float l = L / sqrtf(2.f);
  float t1 = momentCmd.x / l;
  float t2 = momentCmd.y / l;
  float t3 = -momentCmd.z / kappa;
  float t4 = collThrustCmd;

  cmd.desiredThrustsN[0] = (t1 + t2 + t3 + t4) / 4.f;  // front left  - f1
  cmd.desiredThrustsN[1] = (-t1 + t2 - t3 + t4) / 4.f; // front right - f2
  cmd.desiredThrustsN[2] = (t1 - t2 - t3 + t4) / 4.f; // rear left   - f4
  cmd.desiredThrustsN[3] = (-t1 - t2 + t3 + t4) / 4.f; // rear right  - f3

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  ////////////////Comparable Python Code:////////////////

  // MOI = np.array([0.005, 0.005, 0.01])
  // moment_cmd = MOI * np.multiply(self.body_rate_kp, (body_rate_cmd - body_rate))
  // return moment_cmd

  //////////////////////////////////////////////////////

  /*V3F I;
  I.x = Ixx;
  I.y = Iyy;
  I.z = Izz;
  momentCmd = I * kpPQR * (pqrCmd - pqr);*/

  V3F rate_error = pqrCmd - pqr;
  momentCmd = V3F(Ixx, Iyy, Izz)*kpPQR*rate_error;
  

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  ////////////////Comparable Python Code:////////////////

  //if thrust_cmd > 0.:
  //    c = -thrust_cmd / DRONE_MASS_KG;
  //    b_x_c, b_y_c = np.clip(acceleration_cmd / c, -1., 1)
  //    rot_mat = euler2RM(*attitude)
  //    b_x_err = b_x_c - rot_mat[0, 2]
  //    b_x_p_term = self.roll_pitch_kp_roll * b_x_err

  //    b_y = rot_mat[1, 2]
  //    b_y_err = b_y_c - b_y
  //    b_y_p_term = self.roll_pitch_kp_pitch * b_y_err

  //    rot_mat1 = np.array([[rot_mat[1, 0], -rot_mat[0, 0]], [rot_mat[1, 1], -rot_mat[0, 1]]] ) / rot_mat[2, 2]
  //    rot_rate = np.matmul(rot_mat1, np.array([b_x_p_term, b_y_p_term]).T)
  //    p_c = rot_rate[0]
  //    q_c = rot_rate[1]
  //    return np.array([p_c, q_c])
  //else:
  //    return np.array([0., 0.])

  //////////////////////////////////////////////////////
  
  if (collThrustCmd > 0.0) {
      float c = -collThrustCmd / mass;
      float b_x_cmd = CONSTRAIN(accelCmd.x / c, -maxTiltAngle, maxTiltAngle);
      float b_x_err = b_x_cmd - R(0, 2);
      float b_x_p_term = kpBank * b_x_err;

      float b_y_cmd = CONSTRAIN(accelCmd.y / c, -maxTiltAngle, maxTiltAngle);
      float b_y_err = b_y_cmd - R(1, 2);
      float b_y_p_term = kpBank * b_y_err;

      pqrCmd.x = (R(1, 0) * b_x_p_term - R(0, 0) * b_y_p_term) / R(2, 2);
      pqrCmd.y = (R(1, 1) * b_x_p_term - R(0, 1) * b_y_p_term) / R(2, 2);
  }
  else {
      pqrCmd.x = 0.0;
      pqrCmd.y = 0.0;
  }

  pqrCmd.z = 0.0;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

  ////////////////Comparable Python Code:////////////////
  //z_err = altitude_cmd - altitude
  //    z_err_dot = vertical_velocity_cmd - vertical_velocity
  //    b_z = euler2RM(*attitude)[2, 2]

  //    u_1 = self.altitude_kp * z_err + self.altitude_kd * z_err_dot + acceleration_ff
  //    acc = (u_1 - GRAVITY) / b_z

  //    thrust = DRONE_MASS_KG * acc


  //////////////////////////////////////////////////////
  
  float z_err = posZCmd - posZ;
  float p_term = kpPosZ * z_err;

  float z_dot_err = velZCmd - velZ;
  integratedAltitudeError += z_err * dt;


  float d_term = kpVelZ * z_dot_err + velZ;
  float i_term = KiPosZ * integratedAltitudeError;
  float b_z = R(2, 2);

  float u_1_bar = p_term + d_term + i_term + accelZCmd;

  float acc = (u_1_bar - CONST_GRAVITY) / b_z;

  thrust = -mass * CONSTRAIN(acc, -maxAscentRate / dt, maxAscentRate / dt);


  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  ////////////////Comparable Python Code:////////////////

  //err_p = local_position_cmd - local_position
  //err_dot = local_velocity_cmd - local_velocity

  //return self.lateral_kp * err_p + self.lateral_kd * err_dot + acceleration_ff


  //////////////////////////////////////////////////////

  V3F kpPos;
  kpPos.x = kpPosXY;
  kpPos.y = kpPosXY;
  kpPos.z = 0.f;

  V3F kpVel;
  kpVel.x = kpVelXY;
  kpVel.y = kpVelXY;
  kpVel.z = 0.f;

  V3F capVelCmd;
  if (velCmd.mag() > maxSpeedXY) {
      capVelCmd = velCmd.norm() * maxSpeedXY;
  }
  else {
      capVelCmd = velCmd;
  }

  accelCmd = kpPos * (posCmd - pos) + kpVel * (capVelCmd - vel) + accelCmd;

  if (accelCmd.mag() > maxAccelXY) {
      accelCmd = accelCmd.norm() * maxAccelXY;
  }
  

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  
  ////////////////Comparable Python Code:////////////////

  //yaw_error = yaw_cmd - yaw

  //if yaw_error > np.pi:
  //      yaw_error = yaw_error - 2.0 * np.pi
  //elif yaw_error < -np.pi :
  //      yaw_error = yaw_error + 2.0 * np.pi

  //r_c = self.yaw_kp * yaw_error

  //return r_c

  //////////////////////////////////////////////////////

  float yaw_cmd_2_pi = 0;
  if (yawCmd > 0) {
      yaw_cmd_2_pi = fmodf(yawCmd, 2 * F_PI);
  }
  else {
      yaw_cmd_2_pi = -fmodf(-yawCmd, 2 * F_PI);
  }
  float err = yaw_cmd_2_pi - yaw;
  if (err > F_PI) {
      err -= 2 * F_PI;
  } if (err < -F_PI) {
      err += 2 * F_PI;
  }
  yawRateCmd = kpYaw * err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
