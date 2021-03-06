#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"

#include "transform.h"
#include <math.h>

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == HERMITE) && (m_AngleRepresentation == EULER)) // NEW
    HermiteInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == HERMITE) && (m_AngleRepresentation == QUATERNION)) // NEW
    HermiteInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  double rX[4][4], rY[4][4], rZ[4][4], rZYX[4][4], rYX[4][4];

  // Set each rotation matrix
  rotationX(rX, angles[0]);
  rotationY(rY, angles[1]);
  rotationZ(rZ, angles[2]);

  // Rotation matrix multiplications
  matrix_mult(rY, rX, rYX);
  matrix_mult(rZ, rYX, rZYX);

  // Feed the result back to R
  R[0] = rZYX[0][0]; R[1] = rZYX[0][1]; R[2] = rZYX[0][2];
  R[3] = rZYX[1][0]; R[4] = rZYX[1][1]; R[5] = rZYX[1][2];
  R[6] = rZYX[2][0]; R[7] = rZYX[2][1]; R[8] = rZYX[2][2];
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        vector v1 = startPosture->bone_rotation[bone];
        vector v2 = endPosture->bone_rotation[bone];
        vector a, b;

        // Set a (second Bezier control point)
        if (startKeyframe == 0) {
          Posture * nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
          vector v3 = nextPosture->bone_rotation[bone];
          vector tmp = LerpEuler(2.0, v3, v2);
          a = LerpEuler(1.0/3.0, v1, tmp);
        } else {
          Posture * prevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          vector vprev = prevPosture->bone_rotation[bone];
          vector tmp = LerpEuler(2.0, vprev, v1);
          vector tmp2 = LerpEuler(0.5, tmp, v2);
          a = LerpEuler(1.0/3.0, v1, tmp2);
        }

        // Set b (third Bezier control point)
        if (endKeyframe + N + 1 > inputLength) {
          Posture * prevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          vector vprev = prevPosture->bone_rotation[bone];
          vector tmp = LerpEuler(2.0, vprev, v1);
          b = LerpEuler(1.0/3.0, v2, tmp);
        } else {
          Posture * nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
          vector vnext = nextPosture->bone_rotation[bone];
          vector tmp = LerpEuler(2.0, v1, v2);
          vector tmp2 = LerpEuler(0.5, tmp, vnext);
          b = LerpEuler(-1.0/3.0, v2, tmp2);
        }

        interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, v1, a, b, v2);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        Quaternion<double> startQ, endQ, interpQ;
        
        // convert Euler to quaternion
        double arrRotS[3] = {startPosture->bone_rotation[bone].x(), startPosture->bone_rotation[bone].y(), startPosture->bone_rotation[bone].z()};
        Euler2Quaternion(arrRotS, startQ);
        double arrRotE[3] = {endPosture->bone_rotation[bone].x(), endPosture->bone_rotation[bone].y(), endPosture->bone_rotation[bone].z()};
        Euler2Quaternion(arrRotE, endQ);
        
        // interpolate
        interpQ = Slerp(t, startQ, endQ);

        // If SLERP is not applicable, use linear interpolation
        if (isnan(interpQ.Gets())) {
          interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
        } else {
          // convert quaternion to Euler
          double angles[3];
          Quaternion2Euler(interpQ, angles);
          interpolatedPosture.bone_rotation[bone] = angles;
        }
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        Quaternion<double> startQ, endQ, a, b, interpQ;
        
        // convert Euler to quaternion
        double arrRotS[3] = {startPosture->bone_rotation[bone].x(), startPosture->bone_rotation[bone].y(), startPosture->bone_rotation[bone].z()};
        Euler2Quaternion(arrRotS, startQ);
        double arrRotE[3] = {endPosture->bone_rotation[bone].x(), endPosture->bone_rotation[bone].y(), endPosture->bone_rotation[bone].z()};
        Euler2Quaternion(arrRotE, endQ);

        // Get a (second control point)
        if (startKeyframe == 0) {
          Posture * q3Posture = pInputMotion->GetPosture(endKeyframe + N + 1);
          double arrRot3[3] = {q3Posture->bone_rotation[bone].x(), q3Posture->bone_rotation[bone].y(), q3Posture->bone_rotation[bone].z()};
          Quaternion<double> q3;
          Euler2Quaternion(arrRot3, q3);
          Quaternion<double> intermediate = Double(q3, endQ);
          a = Slerp(1.0/3.0, startQ, intermediate);
        } else {
          Posture * prevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          double arrRotPrev[3] = {prevPosture->bone_rotation[bone].x(), prevPosture->bone_rotation[bone].y(), prevPosture->bone_rotation[bone].z()};
          Quaternion<double> prev;
          Euler2Quaternion(arrRotPrev, prev);
          Quaternion<double> intermediate = Double(prev, startQ);
          Quaternion<double> aa = Slerp(0.5, intermediate, endQ);

          a = Slerp(1.0/3.0, startQ, aa);
        }
        
        // Get b (third control point)
        if (endKeyframe + N + 1 > inputLength) {
          Posture * PrevPrevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          double arrRotPrevPrev[3] = {PrevPrevPosture->bone_rotation[bone].x(), PrevPrevPosture->bone_rotation[bone].y(), PrevPrevPosture->bone_rotation[bone].z()};
          Quaternion<double> qPrevPrev;
          Euler2Quaternion(arrRotPrevPrev, qPrevPrev);
          Quaternion<double> intermediate = Double(qPrevPrev, startQ);
          b = Slerp(1.0/3.0, endQ, intermediate);
        } else {
          Posture * nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
          double arrRotNext[3] = {nextPosture->bone_rotation[bone].x(), nextPosture->bone_rotation[bone].y(), nextPosture->bone_rotation[bone].z()};
          Quaternion<double> next;
          Euler2Quaternion(arrRotNext, next);
          Quaternion<double> intermediate = Double(startQ, endQ);
          Quaternion<double> aa = Slerp(0.5, intermediate, next);
          b = Slerp(-1.0/3.0, endQ, aa);
        }
        
        // interpolate
        interpQ = DeCasteljauQuaternion(t, startQ, a, b, endQ);

        // If SLERP is not applicable, use linear interpolation
        if (isnan(interpQ.Gets())) {
          interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
        } else {
          // convert quaternion to Euler
          double angles[3];
          Quaternion2Euler(interpQ, angles);
          interpolatedPosture.bone_rotation[bone] = angles;
        }
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  double R[9];
  Euler2Rotation(angles, R);
  q = Quaternion<double>::Matrix2Quaternion(R);
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  double R[9];
  q.Quaternion2Matrix(R);
  Rotation2Euler(R, angles);
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  qStart.Normalize();
  qEnd_.Normalize();

  // Use the shortest rotation
  double dot = qStart.dot(qEnd_);
  if (dot < 0.0) {
    qStart.neg();
    dot = -dot;
  }

  double theta = acos(dot);
  return qStart*(sin((1.f-t)*theta)/sin(theta)) + qEnd_*(sin(t*theta)/sin(theta));
}

vector Interpolator::LerpEuler(double t, vector & vStart, vector & vEnd_)
{
  return vStart + (vEnd_ - vStart) * t;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  return Slerp(2.0, p, q);
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  vector q0, q1, q2, r0, r1;

  q0 = LerpEuler(t, p0, p1);
  q1 = LerpEuler(t, p1, p2);
  q2 = LerpEuler(t, p2, p3);
  r0 = LerpEuler(t, q0, q1);
  r1 = LerpEuler(t, q1, q2);

  return LerpEuler(t, r0, r1);
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  Quaternion<double> q0, q1, q2, r0, r1;

  q0 = Slerp(t, p0, p1);
  q1 = Slerp(t, p1, p2);
  q2 = Slerp(t, p2, p3);
  r0 = Slerp(t, q0, q1);
  r1 = Slerp(t, q1, q2);

  return Slerp(t, r0, r1);
}

void Interpolator::HermiteInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        vector v1 = startPosture->bone_rotation[bone];
        vector v2 = endPosture->bone_rotation[bone];
        vector vprev, vnext;

        // Set a (second Hermite control point)
        if (startKeyframe == 0) // when there is no previous frame
          vprev = LerpEuler(2.0, v2, v1);
        else {
          Posture * prevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          vprev = prevPosture->bone_rotation[bone];
        }

        // Set b (third Hermite control point)
        if (endKeyframe + N + 1 > inputLength) // when there is no next frame
          vnext = LerpEuler(2.0, v1, v2);
        else {
          Posture * nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
          vnext = nextPosture->bone_rotation[bone];
        }

        interpolatedPosture.bone_rotation[bone] = HermiteEuler(t, vprev, v1, v2, vnext);
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::HermiteInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++) {
        Quaternion<double> startQ, endQ, qprev, qnext, interpQ;
        
        // convert Euler to quaternion
        double arrRotS[3] = {startPosture->bone_rotation[bone].x(), startPosture->bone_rotation[bone].y(), startPosture->bone_rotation[bone].z()};
        Euler2Quaternion(arrRotS, startQ);
        double arrRotE[3] = {endPosture->bone_rotation[bone].x(), endPosture->bone_rotation[bone].y(), endPosture->bone_rotation[bone].z()};
        Euler2Quaternion(arrRotE, endQ);

        // Get a (second control point)
        if (startKeyframe == 0)
          qprev = Double(endQ, startQ);
        else {
          Posture * prevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          double arrRotPrev[3] = {prevPosture->bone_rotation[bone].x(), prevPosture->bone_rotation[bone].y(), prevPosture->bone_rotation[bone].z()};
          Euler2Quaternion(arrRotPrev, qprev);
        }
        
        // Get b (third control point)
        if (endKeyframe + N + 1 > inputLength)
          qnext = Double(startQ, endQ);
        else {
          Posture * nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
          double arrRotNext[3] = {nextPosture->bone_rotation[bone].x(), nextPosture->bone_rotation[bone].y(), nextPosture->bone_rotation[bone].z()};
          Euler2Quaternion(arrRotNext, qnext);
        }
        
        // interpolate
        interpQ = HermiteQuaternion(t, qprev, startQ, endQ, qnext);

        // If SLERP is not applicable, use linear interpolation
        if (isnan(interpQ.Gets())) {
          interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;
        } else {
          // convert quaternion to Euler
          double angles[3];
          Quaternion2Euler(interpQ, angles);
          interpolatedPosture.bone_rotation[bone] = angles;
        }
      }

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

vector Interpolator::HermiteEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // Hermite basis
  double basis[4][4] = {
    { 2, -2,  1,  1},
    {-3,  3, -2, -1},
    { 0,  0,  1,  0},
    { 1,  0,  0,  0}
  };

  // control point matrix
  double control[4][4] = {
    { p0.x(), p0.y(), p0.z(), 0},
    { p1.x(), p1.y(), p1.z(), 0},
    { p2.x(), p2.y(), p2.z(), 0},
    { p3.x(), p3.y(), p3.z(), 0},
  };

  double rhs[4][4];
  matrix_mult(basis, control, rhs);

  double u_1 = t;
  double u_2 = pow(t, 2.0);
  double u_3 = pow(t, 3.0);

  vector result(
    u_3*rhs[0][0]+u_2*rhs[1][0]+u_1*rhs[2][0]+1.0*rhs[3][0],
    u_3*rhs[0][1]+u_2*rhs[1][1]+u_1*rhs[2][1]+1.0*rhs[3][1],
    u_3*rhs[0][2]+u_2*rhs[1][2]+u_1*rhs[2][2]+1.0*rhs[3][2]
  );

  return result;
}

Quaternion<double> Interpolator::HermiteQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // Hermite basis
  double basis[4][4] = {
    { 2, -2,  1,  1},
    {-3,  3, -2, -1},
    { 0,  0,  1,  0},
    { 1,  0,  0,  0}
  };

  // control point matrix
  double control[4][4] = {
    { p0.Gets(), p0.Getx(), p0.Gety(), p0.Getz()},
    { p1.Gets(), p1.Getx(), p1.Gety(), p1.Getz()},
    { p2.Gets(), p2.Getx(), p2.Gety(), p2.Getz()},
    { p3.Gets(), p3.Getx(), p3.Gety(), p3.Getz()},
  };

  double rhs[4][4];
  matrix_mult(basis, control, rhs);

  double u_1 = t;
  double u_2 = pow(t, 2.0);
  double u_3 = pow(t, 3.0);

  Quaternion<double> result(
    u_3*rhs[0][0]+u_2*rhs[1][0]+u_1*rhs[2][0]+1.0*rhs[3][0],
    u_3*rhs[0][1]+u_2*rhs[1][1]+u_1*rhs[2][1]+1.0*rhs[3][1],
    u_3*rhs[0][2]+u_2*rhs[1][2]+u_1*rhs[2][2]+1.0*rhs[3][2],
    u_3*rhs[0][3]+u_2*rhs[1][3]+u_1*rhs[2][3]+1.0*rhs[3][3]
  );

  return result;
}


