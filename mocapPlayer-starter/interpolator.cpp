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
          Quaternion<double> intermediate = Slerp(2.0, q3, endQ);
          a = Slerp(1.0/3.0, startQ, intermediate);
        } else {
          Posture * prevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          double arrRotPrev[3] = {prevPosture->bone_rotation[bone].x(), prevPosture->bone_rotation[bone].y(), prevPosture->bone_rotation[bone].z()};
          Quaternion<double> prev;
          Euler2Quaternion(arrRotPrev, prev);
          Quaternion<double> intermediate = Slerp(2.0, prev, startQ);
          Quaternion<double> aa = Slerp(0.5, intermediate, endQ);

          a = Slerp(1.0/3.0, startQ, aa);
        }
        
        // Get b (third control point)
        if (endKeyframe + N + 1 > inputLength) {
          Posture * PrevPrevPosture = pInputMotion->GetPosture(startKeyframe - (N + 1));
          double arrRotPrevPrev[3] = {PrevPrevPosture->bone_rotation[bone].x(), PrevPrevPosture->bone_rotation[bone].y(), PrevPrevPosture->bone_rotation[bone].z()};
          Quaternion<double> qPrevPrev;
          Euler2Quaternion(arrRotPrevPrev, qPrevPrev);
          Quaternion<double> intermediate = Slerp(2.0, qPrevPrev, startQ);
          b = Slerp(1.0/3.0, endQ, intermediate);
        } else {
          Posture * nextPosture = pInputMotion->GetPosture(endKeyframe + N + 1);
          double arrRotNext[3] = {nextPosture->bone_rotation[bone].x(), nextPosture->bone_rotation[bone].y(), nextPosture->bone_rotation[bone].z()};
          Quaternion<double> next;
          Euler2Quaternion(arrRotNext, next);
          Quaternion<double> intermediate = Slerp(2.0, startQ, endQ);
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
#include <iostream>
Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  Quaternion<double> q1, q2;
  qStart.Normalize();
  qEnd_.Normalize();
  double theta = acos(
                  qStart.Gets() * qEnd_.Gets() +
                  qStart.Getx() * qEnd_.Getx() +
                  qStart.Gety() * qEnd_.Gety() +
                  qStart.Getz() * qEnd_.Getz() );

  return qStart*(sin((1.f-t)*theta)/sin(theta)) + qEnd_*(sin(t*theta)/sin(theta));
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  return result;
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

