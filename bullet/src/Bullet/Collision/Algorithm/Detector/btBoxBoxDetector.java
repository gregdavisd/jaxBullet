/*
 * Box-Box collision detection re-distributed under the ZLib license with permission from Russell L. Smith
 * Original version is from Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org
 *
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision.Algorithm.Detector;

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.btIDebugDraw;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.FLT_MAX;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btAtan2;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import Bullet.LinearMath.btVector4;
import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.vecmath.FloatPointer;
import static javax.vecmath.VecMath.different_epsilon;

/**
 * btBoxBoxDetector wraps the ODE box-box collision detector re-distributed
 * under the Zlib license with permission from Russell L. Smith
 *
 * @author Gregery Barton
 */
public class btBoxBoxDetector extends btDiscreteCollisionDetectorInterface
 implements Serializable {

 private static final float dInfinity = FLT_MAX;
 private static final long serialVersionUID = 1L;

 private static void sub3(float[] a, float[] b, float[] c) {
  a[0] = b[0] - c[0];
  a[1] = b[1] - c[1];
  a[2] = b[2] - c[2];
 }

 private static float dDOT41(FloatPointer a, float[] b) {
  return ((a).get(0) * (b)[(0)] + (a).get(4) * (b)[1] + (a).get(2 * (4))
   * (b)[(2 * (1))]);
 }

 private static float dDOT41(float[] a, float[] b) {
  return ((a)[(0)] * (b)[(0)] + (a)[(4)] * (b)[1] + (a)[(2 * (4))] * (b)[(2
   * (1))]);
 }

 private static float dDOT44(FloatPointer a, FloatPointer b) {
  return ((a).get(0) * (b).get(0) + (a).get(4) * (b).get(4) + (a).get(2 * (4))
   * (b).get(2 * (4)));
 }

 private static float dDOT44(FloatPointer a, float[] b) {
  return ((a).get(0) * (b)[(0)] + (a).get(4) * (b)[(4)] + (a).get(2 * (4))
   * (b)[(2 * (4))]);
 }

 private static float dDOT44(float[] a, float[] b) {
  return ((a)[(0)] * (b)[(0)] + (a)[(4)] * (b)[(4)] + (a)[(2 * (4))] * (b)[(2
   * (4))]);
 }

 private static float dDOT44(float[] a, FloatPointer b) {
  return ((a)[(0)] * (b).get(0) + (a)[(4)] * (b).get(4) + (a)[(2 * (4))] * (b)
   .get(2 * 4));
 }

 private static float dDOT(float[] a, FloatPointer b) {
  return ((a)[(0)] * (b).get(0) + (a)[(1)] * (b).get(1) + (a)[(2 * (1))] * (b)
   .get(2 * (1)));
 }

 private static float dDOT(FloatPointer a, float[] b) {
  return ((a).get(0) * (b)[0] + (a).get(1) * (b)[1] + (a).get(2 * (1)) * (b)[2
   * (1)]);
 }

 private static float dDOT(float[] a, float[] b) {
  return ((a)[(0)] * (b)[0] + (a)[(1)] * (b)[1] + (a)[(2 * (1))] * (b)[2 * (1)]);
 }

 private static float dDOT14(float[] a, FloatPointer b) {
  return ((a)[0] * (b).get(0) + (a)[1] * (b).get(4) + (a)[2 * (1)] * (b).get(2
   * (4)));
 }

 private static void dLineClosestApproach(float[] pa, float[] ua,
  float[] pb, float[] ub,
  float[] alpha, float[] beta) {
  float[] p = new float[3];
  p[0] = pb[0] - pa[0];
  p[1] = pb[1] - pa[1];
  p[2] = pb[2] - pa[2];
  float uaub = dDOT(ua, ub);
  float q1 = dDOT(ua, p);
  float q2 = -dDOT(ub, p);
  float d = 1 - uaub * uaub;
  if (d <= (0.0001f)) {
   // @@@ this needs to be made more robust
   alpha[0] = 0;
   beta[0] = 0;
  } else {
   d = 1.f / d;
   alpha[0] = (q1 + uaub * q2) * d;
   beta[0] = (uaub * q1 + q2) * d;
  }
 }
// find all the intersection points between the 2D rectangle with vertices
// at (+/-h[0],+/-h[1]) and the 2D quadrilateral with vertices (p[0],p[1]),
// (p[2],p[3]),(p[4],p[5]),(p[6],p[7]).
//
// the intersection points are returned as x,y pairs in the 'ret' array.
// the number of intersection points is returned by the function (this will
// be in the range 0 to 8).

 private static int intersectRectQuad2(float[] h, float[] p, float[] ret) {
  // q (and r) contain nq (and nr) coordinate points for the current (and
  // chopped) polygons
  int nq = 4, nr = 0;
  float[] buffer = new float[16];
  FloatPointer q = new FloatPointer(p);
  FloatPointer r = new FloatPointer(ret);
  boolean done = false;
  for (int dir = 0; (!done) && (dir <= 1); dir++) {
   // direction notation: xy[0] = x axis, xy[1] = y axis
   for (int sign = -1; (!done) && (sign <= 1); sign += 2) {
    // chop q along the line xy[dir] = sign*h[dir]
    FloatPointer pq = new FloatPointer(q);
    FloatPointer pr = new FloatPointer(r);
    nr = 0;
    for (int i = nq; i > 0; i--) {
     // go through all points in q and all lines between adjacent points
     if (sign * pq.get(dir) < h[dir]) {
      // this point is inside the chopping line
      pr.set(0, pq.get(0));
      pr.set(1, pq.get(1));
      pr = new FloatPointer(pr, 2);
      nr++;
      if ((nr & 8) != 0) {
       q = r;
       done = true;
       break;
      }
     }
     FloatPointer nextq = (i > 1) ? new FloatPointer(pq, 2) : new FloatPointer(q);
     if ((sign * pq.get(dir) < h[dir]) ^ (sign * nextq.get(dir) < h[dir])) {
      // this line crosses the chopping line
      pr.set(1 - dir, pq.get(1 - dir) + (nextq.get(1 - dir) - pq.get(1 - dir))
       / (nextq.get(dir) - pq.get(dir)) * (sign * h[dir] - pq.get(dir)));
      pr.set(dir, sign * h[dir]);
      pr = new FloatPointer(pr, 2);
      nr++;
      if ((nr & 8) != 0) {
       q = r;
       done = true;
       break;
      }
     }
     pq = new FloatPointer(pq, 2);
    }
    if (!done) {
     q = r;
     r = (q.is_pointing_to(ret)) ? new FloatPointer(buffer) : new FloatPointer(
      ret);
     nq = nr;
    }
   }
  }
  //done:
  if (!q.is_pointing_to(ret)) {
   q.read(ret, nr * 2);
  }
  return nr;
 }

 static void cullPoints2(int n, float p[], int m, int i0, int iret[]) {
  // compute the centroid of the polygon in cx,cy
  int i, j;
  float a, cx, cy, q;
  switch (n) {
   case 1:
    cx = p[0];
    cy = p[1];
    break;
   case 2:
    cx = (0.5f) * (p[0] + p[2]);
    cy = (0.5f) * (p[1] + p[3]);
    break;
   default:
    a = 0;
    cx = 0;
    cy = 0;
    for (i = 0; i < (n - 1); i++) {
     q = p[i * 2] * p[i * 2 + 3] - p[i * 2 + 2] * p[i * 2 + 1];
     a += q;
     cx += q * (p[i * 2] + p[i * 2 + 2]);
     cy += q * (p[i * 2 + 1] + p[i * 2 + 3]);
    }
    q = p[n * 2 - 2] * p[1] - p[0] * p[n * 2 - 1];
    if (btFabs(a + q) > SIMD_EPSILON) {
     a = 1.f / ((3.0f) * (a + q));
    } else {
     a = BT_LARGE_FLOAT;
    }
    cx = a * (cx + q * (p[n * 2 - 2] + p[0]));
    cy = a * (cy + q * (p[n * 2 - 1] + p[1]));
    break;
  }
  // compute the angle of each point w.r.t. the centroid
  float[] A = new float[8];
  for (i = 0; i < n; i++) {
   A[i] = btAtan2(p[i * 2 + 1] - cy, p[i * 2] - cx);
  }
  // search for points that have angles closest to A[i0] + i*(2*pi/m).
  int[] avail = new int[8];
  for (i = 0; i < n; i++) {
   avail[i] = 1;
  }
  avail[i0] = 0;
  iret[0] = i0;
  int i_iret = 0;
  i_iret++;
  for (j = 1; j < m; j++) {
   a = (j) * (2.0f * (float) Math.PI / m) + A[i0];
   if (a > (float) Math.PI) {
    a -= 2.0f * (float) Math.PI;
   }
   float maxdiff = 1e9f;
   float diff;
   iret[i_iret] = i0;			// iret is not allowed to keep this value, but it sometimes does, when diff=#QNAN0
   for (i = 0; i < n; i++) {
    if (avail[i] != 0) {
     diff = btFabs(A[i] - a);
     if (diff > (float) Math.PI) {
      diff = 2.0f * (float) Math.PI - diff;
     }
     assert (Float.isFinite(diff));
     if (diff < maxdiff) {
      maxdiff = diff;
      iret[i_iret] = i;
     }
    }
   }
   assert (iret[i_iret] != i0);	// ensure iret got set
   avail[iret[i_iret]] = 0;
   i_iret++;
  }
 }

 final btBoxShape m_box1;
 final btBoxShape m_box2;

 public btBoxBoxDetector(btBoxShape box1, btBoxShape box2) {
  m_box1 = box1;
  m_box2 = box2;
 }

 void destroy() {
 }

 @Override
 public void getClosestPoints(ClosestPointInput input, Result output,
  btIDebugDraw debugDraw,
  boolean swapResults) {
  final btTransform transformA = input.m_transformA;
  final btTransform transformB = input.m_transformB;
  final float[] R1 = new float[4 * 3];
  final float[] R2 = new float[4 * 3];
  final btVector4 A_j_row = new btVector4();
  final btVector4 B_j_row = new btVector4();
  for (int j = 0; j < 3; j++) {
   transformA.getRow(j, A_j_row);
   transformB.getRow(j, B_j_row);
   R1[0 + 4 * j] = A_j_row.x();
   R2[0 + 4 * j] = B_j_row.x();
   R1[1 + 4 * j] = A_j_row.y();
   R2[1 + 4 * j] = B_j_row.y();
   R1[2 + 4 * j] = A_j_row.z();
   R2[2 + 4 * j] = B_j_row.z();
  }
  float[] normal = new float[3];
  float[] depth = new float[1];
  int[] return_code = new int[1];
  int maxc = 4;
  dBoxBox2(transformA.getOrigin().get(new float[3]),
   R1,
   m_box1.getHalfExtentsWithMargin().scale(2.f).get(new float[3]),
   transformB.getOrigin().get(new float[3]),
   R2,
   m_box2.getHalfExtentsWithMargin().scale(2.f).get(new float[3]),
   normal, depth, return_code,
   maxc,
   output
  );
 }

 int dBoxBox2(float[] p1, float[] R1,
  float[] side1, float[] p2,
  float[] R2, float[] side2,
  float[] normal, float[] depth, int[] return_code,
  int maxc, btDiscreteCollisionDetectorInterface.Result output) {
  int do_maxc = maxc;
  /*
   * c++ macros expanded to inline code
   */
  float fudge_factor = (1.05f);
  final float[] p = new float[3];
  final float[] pp = new float[3];
  final float[] normalC = new float[3];
  FloatPointer normalR = null;
  final float[] A = new float[3];
  final float[] B = new float[3];
  float R11, R12, R13, R21, R22, R23, R31, R32, R33,
   Q11, Q12, Q13, Q21, Q22, Q23, Q31, Q32, Q33, s, s2, l;
  int i, j, invert_normal, code;
  // get vector from centers of box 1 to box 2, relative to box 1
  sub3(p, p2, p1);
// get pp = p relative to body 1
  dMULTIPLY1_331(pp, R1, p);
  calc_half_extents(A, side1, B, side2);
// Rij is R1'*R2, i.e. the relative rotation between R1 and R2
  R11 = dDOT44(R1, R2);
  R12 = dDOT44(R1, new FloatPointer(R2, 1));
  R13 = dDOT44(R1, new FloatPointer(R2, 2));
  R21 = dDOT44(new FloatPointer(R1, 1), R2);
  R22 = dDOT44(new FloatPointer(R1, 1), new FloatPointer(R2, 1));
  R23 = dDOT44(new FloatPointer(R1, 1), new FloatPointer(R2, 2));
  R31 = dDOT44(new FloatPointer(R1, 2), R2);
  R32 = dDOT44(new FloatPointer(R1, 2), new FloatPointer(R2, 1));
  R33 = dDOT44(new FloatPointer(R1, 2), new FloatPointer(R2, 2));
  Q11 = btFabs(R11);
  Q12 = btFabs(R12);
  Q13 = btFabs(R13);
  Q21 = btFabs(R21);
  Q22 = btFabs(R22);
  Q23 = btFabs(R23);
  Q31 = btFabs(R31);
  Q32 = btFabs(R32);
  Q33 = btFabs(R33);
// for all 15 possible separating axes:
//   * see if the axis separates the boxes. if so, return 0.
//   * find the depth of the penetration along the separating axis (s2)
//   * if this is the largest depth so far, record it.
// the normal vector will be set to the separating axis with the smallest
// depth. note: normalR is set to point to a column of R1 or R2 if that is
// the smallest depth normal so far. otherwise normalR is 0 and normalC is
// set to a vector relative to body 1. invert_normal is 1 if the sign of
// the normal should be flipped.
  s = -dInfinity;
  invert_normal = 0;
  code = 0;
// separating axis = u1,u2,u3
  s2 = btFabs(pp[0]) - ((A[0] + B[0] * Q11 + B[1] * Q12 + B[2] * Q13));
  if (s2 > 0) {
   return 0;
  }
  if (s2 > s) {
   s = s2;
   normalR = new FloatPointer(R1, 0);
   invert_normal = ((pp[0]) < 0) ? 1 : 0;
   code = (1);
  }
  s2 = btFabs(pp[1]) - ((A[1] + B[0] * Q21 + B[1] * Q22 + B[2] * Q23));
  if (s2 > 0) {
   return 0;
  }
  if (s2 > s) {
   s = s2;
   normalR = new FloatPointer(R1, 1);
   invert_normal = ((pp[1]) < 0) ? 1 : 0;
   code = (2);
  }
  s2 = btFabs(pp[2]) - ((A[2] + B[0] * Q31 + B[1] * Q32 + B[2] * Q33));
  if (s2 > 0) {
   return 0;
  }
  if (s2 > s) {
   s = s2;
   normalR = new FloatPointer(R1, 2);
   invert_normal = ((pp[2]) < 0) ? 1 : 0;
   code = (3);
  }
// separating axis = v1,v2,v3
  s2 = btFabs(dDOT41(R2, p)) - ((A[0] * Q11 + A[1] * Q21 + A[2] * Q31 + B[0]));
  if (s2 > 0) {
   return 0;
  }
  if (s2 > s) {
   s = s2;
   normalR = new FloatPointer(R2, 0);
   invert_normal = ((dDOT41(R2, p)) < 0) ? 1 : 0;
   code = (4);
  }
  s2 = btFabs(dDOT41(new FloatPointer(R2, 1), p)) - ((A[0] * Q12 + A[1] * Q22
   + A[2] * Q32 + B[1]));
  if (s2 > 0) {
   return 0;
  }
  if (s2 > s) {
   s = s2;
   normalR = new FloatPointer(R2, 1);
   invert_normal = ((dDOT41(new FloatPointer(R2, 1), p)) < 0) ? 1 : 0;
   code = (5);
  }
  s2 = btFabs(dDOT41(new FloatPointer(R2, 2), p)) - ((A[0] * Q13 + A[1] * Q23
   + A[2] * Q33 + B[2]));
  if (s2 > 0) {
   return 0;
  }
  if (s2 > s) {
   s = s2;
   normalR = new FloatPointer(R2, 2);
   invert_normal = ((dDOT41(new FloatPointer(R2, 2), p)) < 0) ? 1 : 0;
   code = (6);
  }
// note: cross product axes need to be scaled when s is computed.
// normal (n1,n2,n3) is relative to box 1.
  final float fudge2 = 1.0e-5f;
  Q11 += fudge2;
  Q12 += fudge2;
  Q13 += fudge2;
  Q21 += fudge2;
  Q22 += fudge2;
  Q23 += fudge2;
  Q31 += fudge2;
  Q32 += fudge2;
  Q33 += fudge2;
  s2 = btFabs(pp[2] * R21 - pp[1] * R31) - ((A[1] * Q31 + A[2] * Q21 + B[1]
   * Q13 + B[2] * Q12));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((0) * (0) + (-R31) * (-R31) + (R21) * (R21));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (0) / l;
    normalC[1] = (-R31) / l;
    normalC[2] = (R21) / l;
    invert_normal = ((pp[2] * R21 - pp[1] * R31) < 0) ? 1 : 0;
    code = (7);
   }
  }
  s2 = btFabs(pp[2] * R22 - pp[1] * R32) - ((A[1] * Q32 + A[2] * Q22 + B[0]
   * Q13 + B[2] * Q11));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((0) * (0) + (-R32) * (-R32) + (R22) * (R22));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (0) / l;
    normalC[1] = (-R32) / l;
    normalC[2] = (R22) / l;
    invert_normal = ((pp[2] * R22 - pp[1] * R32) < 0) ? 1 : 0;
    code = (8);
   }
  }
  s2 = btFabs(pp[2] * R23 - pp[1] * R33) - ((A[1] * Q33 + A[2] * Q23 + B[0]
   * Q12 + B[1] * Q11));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((0) * (0) + (-R33) * (-R33) + (R23) * (R23));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (0) / l;
    normalC[1] = (-R33) / l;
    normalC[2] = (R23) / l;
    invert_normal = ((pp[2] * R23 - pp[1] * R33) < 0) ? 1 : 0;
    code = (9);
   }
  }
  s2 = btFabs(pp[0] * R31 - pp[2] * R11) - ((A[0] * Q31 + A[2] * Q11 + B[1]
   * Q23 + B[2] * Q22));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((R31) * (R31) + (0) * (0) + (-R11) * (-R11));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (R31) / l;
    normalC[1] = (0) / l;
    normalC[2] = (-R11) / l;
    invert_normal = ((pp[0] * R31 - pp[2] * R11) < 0) ? 1 : 0;
    code = (10);
   }
  }
  s2 = btFabs(pp[0] * R32 - pp[2] * R12) - ((A[0] * Q32 + A[2] * Q12 + B[0]
   * Q23 + B[2] * Q21));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((R32) * (R32) + (0) * (0) + (-R12) * (-R12));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (R32) / l;
    normalC[1] = (0) / l;
    normalC[2] = (-R12) / l;
    invert_normal = ((pp[0] * R32 - pp[2] * R12) < 0) ? 1 : 0;
    code = (11);
   }
  }
  s2 = btFabs(pp[0] * R33 - pp[2] * R13) - ((A[0] * Q33 + A[2] * Q13 + B[0]
   * Q22 + B[1] * Q21));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((R33) * (R33) + (0) * (0) + (-R13) * (-R13));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (R33) / l;
    normalC[1] = (0) / l;
    normalC[2] = (-R13) / l;
    invert_normal = ((pp[0] * R33 - pp[2] * R13) < 0) ? 1 : 0;
    code = (12);
   }
  }
  s2 = btFabs(pp[1] * R11 - pp[0] * R21) - ((A[0] * Q21 + A[1] * Q11 + B[1]
   * Q33 + B[2] * Q32));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((-R21) * (-R21) + (R11) * (R11) + (0) * (0));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (-R21) / l;
    normalC[1] = (R11) / l;
    normalC[2] = (0) / l;
    invert_normal = ((pp[1] * R11 - pp[0] * R21) < 0) ? 1 : 0;
    code = (13);
   }
  }
  s2 = btFabs(pp[1] * R12 - pp[0] * R22) - ((A[0] * Q22 + A[1] * Q12 + B[0]
   * Q33 + B[2] * Q31));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((-R22) * (-R22) + (R12) * (R12) + (0) * (0));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (-R22) / l;
    normalC[1] = (R12) / l;
    normalC[2] = (0) / l;
    invert_normal = ((pp[1] * R12 - pp[0] * R22) < 0) ? 1 : 0;
    code = (14);
   }
  }
  s2 = btFabs(pp[1] * R13 - pp[0] * R23) - ((A[0] * Q23 + A[1] * Q13 + B[0]
   * Q32 + B[1] * Q31));
  if (s2 > SIMD_EPSILON) {
   return 0;
  }
  l = btSqrt((-R23) * (-R23) + (R13) * (R13) + (0) * (0));
  if (l > SIMD_EPSILON) {
   s2 /= l;
   if (s2 * fudge_factor > s) {
    s = s2;
    normalR = null;
    normalC[0] = (-R23) / l;
    normalC[1] = (R13) / l;
    normalC[2] = (0) / l;
    invert_normal = ((pp[1] * R13 - pp[0] * R23) < 0) ? 1 : 0;
    code = (15);
   }
  }
  if (code == 0) {
   return 0;
  }
// if we get to this point, the boxes interpenetrate. compute the normal
// in global coordinates.
  calc_global_normal(normalR, normal, R1, normalC, invert_normal);
  depth[0] = -s;
// compute contact point(s)
  if (code > 6) {
   // an edge from box 1 touches an edge from box 2.
   // find a point pa on the intersecting edge of box 1
   float[] pa = new float[3];
   float sign;
   for (i = 0; i < 3; i++) {
    pa[i] = p1[i];
   }
   for (j = 0; j < 3; j++) {
    sign = (dDOT14(normal, new FloatPointer(R1, j)) > 0f) ? (1.0f) : (-1.0f);
    for (i = 0; i < 3; i++) {
     pa[i] += sign * A[j] * R1[i * 4 + j];
    }
   }
   // find a point pb on the intersecting edge of box 2
   float[] pb = new float[3];
   for (i = 0; i < 3; i++) {
    pb[i] = p2[i];
   }
   for (j = 0; j < 3; j++) {
    sign = (dDOT14(normal, new FloatPointer(R2, j)) > 0f) ? (-1.0f) : (1.0f);
    for (i = 0; i < 3; i++) {
     pb[i] += sign * B[j] * R2[i * 4 + j];
    }
   }
   float[] alpha = new float[1];
   float[] beta = new float[1];
   float[] ua = new float[3];
   float[] ub = new float[3];
   for (i = 0; i < 3; i++) {
    ua[i] = R1[((code) - 7) / 3 + i * 4];
   }
   for (i = 0; i < 3; i++) {
    ub[i] = R2[((code) - 7) % 3 + i * 4];
   }
   dLineClosestApproach(pa, ua, pb, ub, alpha, beta);
   for (i = 0; i < 3; i++) {
    pa[i] += ua[i] * alpha[0];
   }
   for (i = 0; i < 3; i++) {
    pb[i] += ub[i] * beta[0];
   }
   {
    output.addContactPoint(new btVector3(normal).negate(), new btVector3(pb),
     -depth[0]);
    return_code[0] = code;
   }
   return 1;
  }
// okay, we have a face-something intersection (because the separating
// axis is perpendicular to a face). define face 'a' to be the reference
// face (i.e. the normal vector is perpendicular to this) and face 'b' to be
// the incident face (the closest face of the other box).
  FloatPointer Ra, Rb, pa, pb, Sa, Sb;
  if (code <= 3) {
   Ra = new FloatPointer(R1);
   Rb = new FloatPointer(R2);
   pa = new FloatPointer(p1);
   pb = new FloatPointer(p2);
   Sa = new FloatPointer(A);
   Sb = new FloatPointer(B);
  } else {
   Ra = new FloatPointer(R2);
   Rb = new FloatPointer(R1);
   pa = new FloatPointer(p2);
   pb = new FloatPointer(p1);
   Sa = new FloatPointer(B);
   Sb = new FloatPointer(A);
  }
// nr = normal vector of reference face dotted with axes of incident box.
// anr = absolute values of nr.
  float[] normal2 = new float[3];
  float[] nr = new float[3];
  float[] anr = new float[3];
  calc_face_box_normal(code, normal2, normal, nr, Rb, anr);
// find the largest compontent of anr: this corresponds to the normal
// for the indident face. the other axis numbers of the indicent face
// are stored in a1,a2.
  int lanr, a1, a2;
  if (anr[1] > anr[0]) {
   if (anr[1] > anr[2]) {
    a1 = 0;
    lanr = 1;
    a2 = 2;
   } else {
    a1 = 0;
    a2 = 1;
    lanr = 2;
   }
  } else if (anr[0] > anr[2]) {
   lanr = 0;
   a1 = 1;
   a2 = 2;
  } else {
   a1 = 0;
   a2 = 1;
   lanr = 2;
  }
// compute center point of incident face, in reference-face coordinates
  float[] center = new float[3];
  calc_incident_face_center(nr, lanr, center, pb, pa, Sb, Rb);
// find the normal and non-normal axis numbers of the reference box
  int codeN, code1, code2;
  if (code <= 3) {
   codeN = code - 1;
  } else {
   codeN = code - 4;
  }
  switch (codeN) {
   case 0:
    code1 = 1;
    code2 = 2;
    break;
   case 1:
    code1 = 0;
    code2 = 2;
    break;
   default:
    code1 = 0;
    code2 = 1;
    break;
  }
// find the four corners of the incident face, in reference-face coordinates
  float[] quad = new float[8];	// 2D coordinate of incident face (x,y pairs)
  float c1, c2;
  c1 = dDOT14(center, new FloatPointer(Ra, code1));
  c2 = dDOT14(center, new FloatPointer(Ra, code2));
// optimize this? - we have already computed this data above, but it is not
// stored in an easy-to-index format. for now it's quicker just to recompute
// the four dot products.
  float m11, m12, m21, m22;
  m11 = dDOT44(new FloatPointer(Ra, code1), new FloatPointer(Rb, a1));
  m12 = dDOT44(new FloatPointer(Ra, code1), new FloatPointer(Rb, a2));
  m21 = dDOT44(new FloatPointer(Ra, code2), new FloatPointer(Rb, a1));
  m22 = dDOT44(new FloatPointer(Ra, code2), new FloatPointer(Rb, a2));
  calc_quad(quad, m11, m12, m21, m22, Sb, a1, a2, c1, c2);
// find the size of the reference face
  float[] rect = new float[2];
  rect[0] = Sa.get(code1);
  rect[1] = Sa.get(code2);
// intersect the incident and reference faces
  float[] ret = new float[16];
  int n = intersectRectQuad2(rect, quad, ret);
  if (n < 1) {
   return 0;		// this should never happen
  }
// convert the intersection points into reference-face coordinates,
// and compute the contact position and depth for each point. only keep
// those points that have a positive (penetrating) depth. delete points in
// the 'ret' array as necessary so that 'point' and 'ret' correspond.
  float[] point = new float[3 * 8];		// penetrating contact points
  float[] dep = new float[8];			// depths for those points
  float det1 = 1.f / (m11 * m22 - m12 * m21);
  m11 *= det1;
  m12 *= det1;
  m21 *= det1;
  m22 *= det1;
  int cnum = 0;			// number of penetrating contact points found
  for (j = 0; j < n; j++) {
   float k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
   float k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
   for (i = 0; i < 3; i++) {
    point[cnum * 3 + i]
     = center[i] + k1 * Rb.get(i * 4 + a1) + k2 * Rb.get(i * 4 + a2);
   }
   dep[cnum] = Sa.get(codeN) - dDOT(normal2, new FloatPointer(point, cnum * 3));
   if (dep[cnum] >= 0) {
    ret[cnum * 2] = ret[j * 2];
    ret[cnum * 2 + 1] = ret[j * 2 + 1];
    cnum++;
   }
  }
  if (cnum < 1) {
   return 0;	// this should never happen
  }
// we can't generate more contacts than we actually have
  if (do_maxc > cnum) {
   do_maxc = cnum;
  }
  if (do_maxc < 1) {
   do_maxc = 1;
  }
  if (cnum <= do_maxc) {
   if (code < 4) {
    // we have less contacts than we need, so we use them all
    for (j = 0; j < cnum; j++) {
     float[] pointInWorld = new float[3];
     for (i = 0; i < 3; i++) {
      pointInWorld[i] = point[j * 3 + i] + pa.get(i);
     }
     output.addContactPoint(new btVector3(normal).negate(), new btVector3(
      pointInWorld), -dep[j]);
    }
   } else {
    // we have less contacts than we need, so we use them all
    for (j = 0; j < cnum; j++) {
     float[] pointInWorld = new float[3];
     for (i = 0; i < 3; i++) {
      pointInWorld[i] = point[j * 3 + i] + pa.get(i) - normal[i] * dep[j];
     }
     //pointInWorld[i] = point[j*3+i] + pa[i];
     output.addContactPoint(new btVector3(normal).negate(), new btVector3(
      pointInWorld), -dep[j]);
    }
   }
  } else {
   // we have more contacts than are wanted, some of them must be culled.
   // find the deepest point, it is always the first contact.
   int i1 = 0;
   float maxdepth = dep[0];
   for (i = 1; i < cnum; i++) {
    if (dep[i] > maxdepth) {
     maxdepth = dep[i];
     i1 = i;
    }
   }
   int[] iret = new int[8];
   cullPoints2(cnum, ret, do_maxc, i1, iret);
   for (j = 0; j < do_maxc; j++) {
    float[] posInWorld = new float[3];
    for (i = 0; i < 3; i++) {
     posInWorld[i] = point[iret[j] * 3 + i] + pa.get(i);
    }
    if (code < 4) {
     output.addContactPoint(new btVector3(normal).negate(), new btVector3(
      posInWorld),
      -dep[iret[j]]);
    } else {
     output.addContactPoint(new btVector3(normal).negate(),
      new btVector3(normal).negate().scaleAdd(dep[iret[j]], new btVector3(
       posInWorld)),
      -dep[iret[j]]);
    }
   }
   cnum = do_maxc;
  }
  return_code[0] = code;
  return cnum;
 }

 private void calc_global_normal(FloatPointer normalR, float[] normal,
  float[] R1,
  final float[] normalC, int invert_normal) {
  if (normalR != null) {
   normal[0] = normalR.get(0);
   normal[1] = normalR.get(4);
   normal[2] = normalR.get(8);
  } else {
   {
    (normal)[0] = dDOT((R1), (normalC));
    (normal)[1] = dDOT((new FloatPointer(R1, 4)), (normalC));
    (normal)[2] = dDOT((new FloatPointer(R1, 8)), (normalC));
   }
  }
  if (invert_normal != 0) {
   normal[0] = -normal[0];
   normal[1] = -normal[1];
   normal[2] = -normal[2];
  }
 }

 private void calc_quad(float[] quad, float m11, float m12, float m21, float m22,
  FloatPointer Sb,
  int a1, int a2, float c1, float c2) {
  float k1 = m11 * Sb.get(a1);
  float k2 = m21 * Sb.get(a1);
  float k3 = m12 * Sb.get(a2);
  float k4 = m22 * Sb.get(a2);
  quad[0] = c1 - k1 - k3;
  quad[1] = c2 - k2 - k4;
  quad[2] = c1 - k1 + k3;
  quad[3] = c2 - k2 + k4;
  quad[4] = c1 + k1 + k3;
  quad[5] = c2 + k2 + k4;
  quad[6] = c1 + k1 - k3;
  quad[7] = c2 + k2 - k4;
 }

 private void calc_incident_face_center(float[] nr, int lanr, float[] center,
  FloatPointer pb,
  FloatPointer pa, FloatPointer Sb, FloatPointer Rb) {
  if (nr[lanr] < 0) {
   for (int i = 0; i < 3; i++) {
    center[i] = pb.get(i) - pa.get(i) + Sb.get(lanr) * Rb.get(i * 4 + lanr);
   }
  } else {
   for (int i = 0; i < 3; i++) {
    center[i] = pb.get(i) - pa.get(i) - Sb.get(lanr) * Rb.get(i * 4 + lanr);
   }
  }
 }

 private void calc_face_box_normal(int code, float[] normal2, float[] normal,
  float[] nr,
  FloatPointer Rb, float[] anr) {
  if (code <= 3) {
   normal2[0] = normal[0];
   normal2[1] = normal[1];
   normal2[2] = normal[2];
  } else {
   normal2[0] = -normal[0];
   normal2[1] = -normal[1];
   normal2[2] = -normal[2];
  }
  {
   (nr)[0] = dDOT41((Rb), (normal2));
   (nr)[1] = dDOT41((new FloatPointer(Rb, 1)), (normal2));
   (nr)[2] = dDOT41((new FloatPointer(Rb, 2)), (normal2));
  }
  anr[0] = btFabs(nr[0]);
  anr[1] = btFabs(nr[1]);
  anr[2] = btFabs(nr[2]);
 }

 private void calc_half_extents(float[] A, float[] side1, float[] B,
  float[] side2) {
  // get side lengths / 2
  A[0] = side1[0] * (0.5f);
  A[1] = side1[1] * (0.5f);
  A[2] = side1[2] * (0.5f);
  B[0] = side2[0] * (0.5f);
  B[1] = side2[1] * (0.5f);
  B[2] = side2[2] * (0.5f);
 }

 private void dMULTIPLY1_331(final float[] pp, float[] R1, final float[] p) {
  (pp)[0] = dDOT41(R1, (p));
  (pp)[1] = dDOT41(new FloatPointer(R1, 1), (p));
  (pp)[2] = dDOT41(new FloatPointer(R1, 2), (p));
 }

 /**
  *
  * @author Gregery Barton
  */
 private static class Testbed {

  private static boolean compare_results(List<BoxBoxOutput> expected,
   List<BoxBoxOutput> results) {
   if (expected.size() != results.size()) {
    return false;
   }
   for (int i = 0; i < expected.size(); i++) {
    BoxBoxOutput cpp = expected.get(i);
    BoxBoxOutput jav = results.get(i);
    if (different_epsilon(cpp.depth, jav.depth, 0)) {
     return false;
    }
    if (!cpp.normalOnBInWorld.epsilonEquals(jav.normalOnBInWorld, 0)) {
     return false;
    }
    if (!cpp.pointInWorld.epsilonEquals(jav.pointInWorld, 0)) {
     return false;
    }
   }
   return true;
  }

  private static class BoxBoxInput {

   final btVector3 box1 = new btVector3();
   final btVector3 box2 = new btVector3();
   float maximumDistanceSquared;
   final btTransform transformA = new btTransform();
   final btTransform transformB = new btTransform();
  }

  private static class BoxBoxOutput {

   int ref;
   final btVector3 normalOnBInWorld = new btVector3();
   final btVector3 pointInWorld = new btVector3();
   float depth;
  }

  private static class BoxBoxRun {

   BoxBoxInput input = new BoxBoxInput();
   List<BoxBoxOutput> points = new ArrayList<>(0);
  }

  private static class RunResult extends Result {

   final List<BoxBoxOutput> points;

   public RunResult(List<BoxBoxOutput> points) {
    this.points = points;
   }

   @Override
   public void setShapeIdentifiersA(int partId0, int index0) {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
   }

   @Override
   public void setShapeIdentifiersB(int partId1, int index1) {
    throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
   }

   @Override
   public void addContactPoint(final btVector3 normalOnBInWorld,
    final btVector3 pointInWorld,
    float depth) {
    BoxBoxOutput o = new BoxBoxOutput();
    o.depth = depth;
    o.normalOnBInWorld.set(normalOnBInWorld);
    o.pointInWorld.set(pointInWorld);
    points.add(o);
   }

  }

  /**
   *
   */
  private static void test() {
   List<BoxBoxRun> cpp_data = new ArrayList<>();
   try {
    BufferedReader r
     = new BufferedReader(new FileReader(
      "../cppexamples/getClosesPoints_input.txt"));
    while (true) {
     String start = r.readLine();
     if (start == null) {
      break;
     }
     if (!"-i".equals(start)) {
      throw new AssertionError();
     }
     BoxBoxRun run = new BoxBoxRun();
     run.input.maximumDistanceSquared = Float.parseFloat(r.readLine());
     read_vector3(r, run.input.box1);
     read_vector3(r, run.input.box2);
     read_transform(r, run.input.transformA);
     read_transform(r, run.input.transformB);
     if (!"-o".equals(r.readLine())) {
      throw new AssertionError();
     }
     while (true) {
      r.mark(16);
      String sref = r.readLine();
      if ("-f".equals(sref)) {
       break;
      } else {
       r.reset();
      }
      BoxBoxOutput out = new BoxBoxOutput();
      out.ref = Integer.parseInt(r.readLine());
      read_vector3(r, out.normalOnBInWorld);
      read_vector3(r, out.pointInWorld);
      out.depth = Float.parseFloat(r.readLine());
      run.points.add(out);
     }
     cpp_data.add(run);
    }
   } catch (FileNotFoundException ex) {
    Logger.getLogger(Testbed.class.getName()).log(Level.SEVERE, null, ex);
   } catch (IOException ex) {
    Logger.getLogger(Testbed.class.getName()).log(Level.SEVERE, null, ex);
   }
   System.out.println(cpp_data.size());
   int n = 0;
   for (BoxBoxRun cpp_run : cpp_data) {
    btBoxShape box1 = new btBoxShape(cpp_run.input.box1);
    btBoxShape box2 = new btBoxShape(cpp_run.input.box2);
    btBoxBoxDetector detector = new btBoxBoxDetector(box1, box2);
    ClosestPointInput cpi = new ClosestPointInput();
    cpi.m_maximumDistanceSquared = cpp_run.input.maximumDistanceSquared;
    cpi.m_transformA.set(cpp_run.input.transformA);
    cpi.m_transformB.set(cpp_run.input.transformB);
    List<BoxBoxOutput> java_run = new ArrayList<>();
    detector.getClosestPoints(cpi, new RunResult(java_run), null);
    if (!compare_results(cpp_run.points, java_run)) {
     System.out.println(n);
     compare_results(cpp_run.points, java_run);
     java_run.clear();
     detector.getClosestPoints(cpi, new RunResult(java_run), null);
     System.out.println("wrong ");
     System.exit(1);
    }
    ++n;
   }
   System.out.println("all same");
  }

  private static void read_transform(BufferedReader r, final btTransform t)
   throws
   NumberFormatException, IOException {
   String transA
    = r.readLine().trim() + " " + r.readLine().trim() + " " + r.readLine()
    .trim() + " " + r.readLine()
     .trim();
   String[] split = transA.split("\\s+");
   float[] mv = new float[12];
   for (int i = 0; i < split.length; ++i) {
    String string = split[i];
    mv[i] = Float.parseFloat(string);
   }
   t.setBasis(new btMatrix3x3(mv));
   t.setOrigin(new btVector3(mv[9], mv[10], mv[11]));
  }

  private static void read_vector3(BufferedReader r, final btVector3 v) throws
   NumberFormatException, IOException {
   String transA = r.readLine().trim();
   String[] split = transA.split("\\s+");
   float[] mv = new float[3];
   for (int i = 0; i < split.length; ++i) {
    String string = split[i];
    mv[i] = Float.parseFloat(string);
   }
   v.set(mv);
  }

 }
}
