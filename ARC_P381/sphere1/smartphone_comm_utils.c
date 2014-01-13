#include "exp_v2.h"
#include "smartphone_comm_utils.h"
#include "std_includes.h"
#include "math.h"
#include "spheres_constants.h"

void smtExpV2UARTSendWHETHeader(unsigned char channel, unsigned char len, unsigned char *data, unsigned char cmd) {
  short big_chk = 0;
  het_header het_hdr;
  int i;

  for (i=0; i < len; i++)
    big_chk += data[i];

  // make our HET header
  het_hdr.preamble[0] = 0xAA;
  het_hdr.preamble[1] = 0x55;
  het_hdr.preamble[2] = 0xAA;
  het_hdr.preamble[3] = 0x55;
  het_hdr.chk = big_chk;
  het_hdr.cmd = cmd;
  het_hdr.len = len;

  expv2_uart_send(channel, sizeof(het_header), (unsigned char *)&het_hdr);
  expv2_uart_send(channel, len, data);
}

float smtGetQuaternionMagnitude(float qw) {
  return 2*acos(qw);
}


int smtAtPositionRotation(state_vector error) {
#ifdef ISS_VERSION
  if( (fabs(error[POS_Y]) < TRANSLATION_MARGIN) && (fabs(error[POS_Z]) < TRANSLATION_MARGIN) &&
      (fabs(error[POS_X]) < TRANSLATION_MARGIN) &&
      (fabs(smtGetQuaternionMagnitude(error[QUAT_4])) < QUAT_AXIS_MARGIN))
#else // on ground, only care about roll
    if( (fabs(error[POS_X]) < TRANSLATION_MARGIN) && (fabs(error[POS_Y]) < TRANSLATION_MARGIN) &&
        (fabs(smtGetQuaternionMagnitude(error[QUAT_4])) < QUAT_AXIS_MARGIN))
#endif
      {
        return TRUE;
      } else {
      return FALSE;
    }
}

int smtAtZeroVelocity(state_vector error) {
#ifdef ISS_VERSION
  if( (fabs(error[VEL_X]) < VELOCITY_MARGIN) && (fabs(error[VEL_Y]) < VELOCITY_MARGIN) &&
      (fabs(error[VEL_Z]) < VELOCITY_MARGIN) &&
      (fabs(error[RATE_X]) < RATE_MARGIN) && (fabs(error[RATE_Y]) < RATE_MARGIN) &&
      (fabs(error[RATE_Z]) < RATE_MARGIN) )
#else
    if( (fabs(error[VEL_X]) < VELOCITY_MARGIN) && (fabs(error[VEL_Y]) < VELOCITY_MARGIN) &&
        (fabs(error[RATE_Z]) < RATE_MARGIN) )
#endif
      {
        return TRUE;
      } else {
      return FALSE;
    }
}

// rotates quaternion 1 by quaternion 2 and returns as total (xyzw)
void smtRotateByQuaternion(float x2, float y2, float z2, float w2,
                           float x1, float y1, float z1, float w1, float* answer) {
  // quats are xyzw
  answer[0] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
  answer[1] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
  answer[2] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
  answer[3] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
}

int smtChecksumVerify(unsigned char* buffer, unsigned int len) {
  int i;
  unsigned short bigcheck=0;

  het_header* hdr = (het_header*) buffer;

  if(len != (8 + hdr->len) )
    return FALSE;

  for(i=8; i<len; i++)
    bigcheck += buffer[i];

  if(bigcheck != hdr->chk)
    return FALSE;

  return TRUE;
}

void smtRotatePhonePositionByQuaternion(float q[4], float position[3],
                                        float res[3]) {
  // http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
  float m00, m01, m02, m10, m11, m12, m20, m21, m22;
  float qx, qy, qz, qw;
  float v0, v1, v2;
  qw = q[3];
  qx = q[0];
  qy = q[1];
  qz = q[2];
  v0 = position[0];
  v1 = position[1];
  v2 = position[2];

  m00 = 1 - 2*qy*qy - 2*qz*qz;
  m01 = 2*qx*qy - 2*qz*qw;
  m02 = 2*qx*qz - 2*qy*qw;
  m10 = 2*qx*qy + 2*qz*qw;
  m11 = 1 - 2*qx*qx - 2*qz*qz;
  m12 = 2*qy*qz - 2*qx*qw;
  m20 = 2*qx*qz - 2*qy*qw;
  m21 = 2*qy*qz + 2*qx*qw;
  m22 = 1 - 2*qx*qx - 2*qy*qy;

  res[0] = m00*v0 + m01*v1 + m02*v2;
  res[1] = m10*v0 + m11*v1 + m12*v2;
  res[2] = m20*v0 + m21*v1 + m22*v2;
}

void smtFindQDot(float q[4], float rotVel[3], float qdot[4]) {
  float wx, wy, wz;
  float o00, o01, o02, o03;
  float o10, o11, o12, o13;
  float o20, o21, o22, o23;
  float o30, o31, o32, o33;

  wx = rotVel[0];
  wy = rotVel[1];
  wz = rotVel[2];

  o00 =   0;
  o10 = -wz;
  o20 =  wy;
  o30 = -wx;

  o01 =  wz;
  o11 =   0;
  o21 = -wx;
  o31 = -wy;

  o02 = -wy;
  o12 =  wx;
  o22 =   0;
  o32 = -wz;

  o03 =  wx;
  o13 =  wy;
  o23 =  wz;
  o33 =   0;

  qdot[0] = o00*q[0] + o01*q[1] + o02*q[2] + o03*q[3];
  qdot[1] = o10*q[0] + o11*q[1] + o12*q[2] + o13*q[3];
  qdot[2] = o20*q[0] + o21*q[1] + o22*q[2] + o23*q[3];
  qdot[3] = o30*q[0] + o31*q[1] + o32*q[2] + o33*q[3];
}

void smtQuatMatrixDerivative(float quat[4], float qdot[4], float matrix[3][3]) {
  float qx, qy, qz, qw;
  float qxdot, qydot, qzdot, qwdot;
  float m00dot, m01dot, m02dot, m10dot, m11dot, m12dot, m20dot, m21dot, m22dot;

  qx = quat[0];
  qy = quat[1];
  qz = quat[2];
  qw = quat[3];

  qxdot = qdot[0];
  qydot = qdot[1];
  qzdot = qdot[2];
  qwdot = qdot[3];

  m00dot = -4*(qy*qydot + qz*qzdot);

  m01dot = -2*qz*qwdot - 2*qw*qzdot + 2*qy*qxdot + 2*qx*qydot;

  m02dot = 2*qy*qwdot + 2*qw*qydot + 2*qz*qxdot + 2*qx*qzdot;

  m10dot = 2*qz*qwdot + 2*qw*qzdot + 2*qy*qxdot + 2*qx*qydot;

  m11dot = -4*(qx*qxdot + qz*qzdot);

  m12dot = -2*qx*qwdot - 2*qw*qxdot + 2*qz*qydot + 2*qy*qzdot;

  m20dot =  -2*qy*qwdot - 2*qw*qydot + 2*qz*qxdot + 2*qx*qzdot;

  m21dot = 2*qx*qwdot + 2*qw*qxdot + 2*qz*qydot + 2*qy*qzdot;

  m22dot =  -4*(qx*qxdot + qy*qydot);

  matrix[0][0] = m00dot;
  matrix[0][1] = m01dot;
  matrix[0][2] = m02dot;

  matrix[1][0] = m10dot;
  matrix[1][1] = m11dot;
  matrix[1][2] = m12dot;

  matrix[2][0] = m20dot;
  matrix[2][1] = m21dot;
  matrix[2][2] = m22dot;
}
