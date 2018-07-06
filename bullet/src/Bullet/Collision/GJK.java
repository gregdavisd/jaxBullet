/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software in a
 * product, an acknowledgment in the product documentation would be appreciated
 * but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision;

import static Bullet.Extras.btMinMax.btMax;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btDot;
import static Bullet.LinearMath.btVector3.det;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class GJK implements Serializable {

 static final float GJK_ACCURACY = (0.0001f);
 static final float GJK_MIN_DISTANCE = (0.0001f);
 static final float GJK_DUPLICATED_EPS = (0.0001f);
 static final float GJK_SIMPLEX2_EPS = 0.0f;
 static final float GJK_SIMPLEX3_EPS = 0.0f;
 static final float GJK_SIMPLEX4_EPS = 0.0f;
 static final int GJK_MAX_ITERATIONS = 128;
 private static final int[] imd3 = new int[]{1, 2, 0};

 /*
  * Types
  */
 static class sSV {

  final btVector3 d = new btVector3();
  final btVector3 w = new btVector3();
 }

 static class sSimplex {

  final sSV[] c = new sSV[4];
  float[] p = new float[4];
  int rank;
 };

 static enum eStatus {
  Valid,
  Inside,
  Failed
 }
 /*
  * Fields
  */
 final MinkowskiDiff m_shape = new MinkowskiDiff();
 final btVector3 m_ray = new btVector3();
 float m_distance;
 final sSimplex[] m_simplices = new sSimplex[]{new sSimplex(), new sSimplex()};
 final sSV[] m_store = new sSV[]{new sSV(), new sSV(), new sSV(), new sSV()};
 final sSV[] m_free = new sSV[4];
 int m_nfree;
 int m_current;
 sSimplex m_simplex;
 GJK.eStatus m_status;

 /*
  * Methods
  */
 GJK() {
  initialize();
 }

 final void initialize() {
  m_nfree = 0;
  m_status = GJK.eStatus.Failed;
  m_current = 0;
  m_distance = 0;
 }

 GJK.eStatus evaluate(MinkowskiDiff shapearg, final btVector3 guess) {
  int iterations = 0;
  float sqdist;
  float alpha = 0;
  btVector3[] lastw = new btVector3[4];
  init(lastw);
  int clastw = 0;
  /*
   * initialize solver
   */
  m_free[0] = m_store[0];
  m_free[1] = m_store[1];
  m_free[2] = m_store[2];
  m_free[3] = m_store[3];
  m_nfree = 4;
  m_current = 0;
  m_status = eStatus.Valid;
  m_shape.set(shapearg);
  m_distance = 0;
  /*
   * initialize simplex
   */
  m_simplices[0].rank = 0;
  m_ray.set(guess);
  float sqrl = m_ray.lengthSquared();
  appendvertice(m_simplices[0],
   (sqrl > 0 ? new btVector3(m_ray).negate() : new btVector3(1, 0, 0)));
  m_simplices[0].p[0] = 1;
  m_ray.set(m_simplices[0].c[0].w);
  sqdist = sqrl;
  lastw[0].set(m_ray);
  lastw[1].set(m_ray);
  lastw[2].set(m_ray);
  lastw[3].set(m_ray);
  /*
   * Loop
   */
  do {
   int next = 1 - m_current;
   sSimplex cs = m_simplices[m_current];
   sSimplex ns = m_simplices[next];
   /*
    * Check zero
    */
   float rl = m_ray.length();
   if (rl < GJK_MIN_DISTANCE) {/*
     * Touching or inside
     */
    m_status = eStatus.Inside;
    break;
   }
   /*
    * Append new vertice in -'v' direction
    */
   appendvertice(cs, new btVector3(m_ray).negate());
   final btVector3 w = cs.c[cs.rank - 1].w;
   boolean found = false;
   for (int i = 0; i < 4; ++i) {
    if (w.distanceSquared(lastw[i]) < GJK_DUPLICATED_EPS) {
     found = true;
     break;
    }
   }
   if (found) {/*
     * Return old simplex
     */
    removevertice(m_simplices[m_current]);
    break;
   } else {/*
     * Update lastw
     */
    lastw[(clastw = (clastw + 1) & 3)].set(w);
   }
   /*
    * Check for termination
    */
   float omega = btDot(m_ray, w) / rl;
   alpha = btMax(omega, alpha);
   if (((rl - alpha) - (GJK_ACCURACY * rl)) <= 0) {/*
     * Return old simplex
     */
    removevertice(m_simplices[m_current]);
    break;
   }
   /*
    * Reduce simplex
    */
   float[] weights = new float[4];
   int[] mask = new int[1];
   switch (cs.rank) {
    case 2:
     sqdist = projectorigin(cs.c[0].w,
      cs.c[1].w,
      weights, mask);
     break;
    case 3:
     sqdist = projectorigin(cs.c[0].w,
      cs.c[1].w,
      cs.c[2].w,
      weights, mask);
     break;
    case 4:
     sqdist = projectorigin(cs.c[0].w,
      cs.c[1].w,
      cs.c[2].w,
      cs.c[3].w,
      weights, mask);
     break;
    default:
     assert (false);
   }
   if (sqdist >= 0) {/*
     * Valid
     */
    ns.rank = 0;
    m_ray.setZero();
    m_current = next;
    for (int i = 0, ni = cs.rank; i < ni; ++i) {
     if ((mask[0] & (1 << i)) != 0) {
      ns.c[ns.rank] = cs.c[i];
      ns.p[ns.rank++] = weights[i];
      m_ray.add(new btVector3(cs.c[i].w).scale(weights[i]));
     } else {
      m_free[m_nfree++] = cs.c[i];
     }
    }
    if (mask[0] == 15) {
     m_status = eStatus.Inside;
    }
   } else {/*
     * Return old simplex
     */
    removevertice(m_simplices[m_current]);
    break;
   }
   m_status = ((++iterations) < GJK_MAX_ITERATIONS) ? m_status : eStatus.Failed;
  } while (m_status == eStatus.Valid);
  m_simplex = m_simplices[m_current];
  switch (m_status) {
   case Valid:
    m_distance = m_ray.length();
    break;
   case Inside:
    m_distance = 0;
    break;
   default: {
   }
  }
  return (m_status);
 }

 boolean encloseOrigin() {
  switch (m_simplex.rank) {
   case 1: {
    for (int i = 0; i < 3; ++i) {
     final btVector3 axis = new btVector3();
     axis.setElement(i, 1.0f);
     appendvertice(m_simplex, axis);
     if (encloseOrigin()) {
      return (true);
     }
     removevertice(m_simplex);
     appendvertice(m_simplex, new btVector3(axis).negate());
     if (encloseOrigin()) {
      return (true);
     }
     removevertice(m_simplex);
    }
   }
   break;
   case 2: {
    final btVector3 d = new btVector3(m_simplex.c[1].w).sub(m_simplex.c[0].w);
    for (int i = 0; i < 3; ++i) {
     final btVector3 axis = new btVector3();
     axis.setElement(i, 1.0f);
     final btVector3 p = new btVector3(d).cross(axis);
     if (p.lengthSquared() > 0) {
      appendvertice(m_simplex, p);
      if (encloseOrigin()) {
       return (true);
      }
      removevertice(m_simplex);
      appendvertice(m_simplex, new btVector3(p).negate());
      if (encloseOrigin()) {
       return (true);
      }
      removevertice(m_simplex);
     }
    }
   }
   break;
   case 3: {
    final btVector3 n = new btVector3(m_simplex.c[1].w).sub(m_simplex.c[0].w)
     .cross(
      new btVector3(m_simplex.c[2].w).sub(m_simplex.c[0].w));
    if (n.lengthSquared() > 0) {
     appendvertice(m_simplex, n);
     if (encloseOrigin()) {
      return (true);
     }
     removevertice(m_simplex);
     appendvertice(m_simplex, new btVector3(n).negate());
     if (encloseOrigin()) {
      return (true);
     }
     removevertice(m_simplex);
    }
   }
   break;
   case 4: {
    if (btFabs(det(new btVector3(m_simplex.c[0].w).sub(m_simplex.c[3].w),
     new btVector3(m_simplex.c[1].w).sub(m_simplex.c[3].w),
     new btVector3(m_simplex.c[2].w).sub(m_simplex.c[3].w))) > 0) {
     return (true);
    }
   }
   break;
  }
  return (false);
 }

 /*
  * Internals
  */
 void getsupport(final btVector3 d, sSV sv) {
  sv.d.set(d).scale(1.0f / d.length());
  sv.w.set(m_shape.support(sv.d));
 }

 void removevertice(sSimplex simplex) {
  m_free[m_nfree++] = simplex.c[--simplex.rank];
 }

 void appendvertice(sSimplex simplex, final btVector3 v) {
  simplex.p[simplex.rank] = 0;
  simplex.c[simplex.rank] = m_free[--m_nfree];
  getsupport(v, simplex.c[simplex.rank++]);
 }

 static float projectorigin(final btVector3 a, final btVector3 b, float[] w,
  int[] m) {
  final btVector3 d = new btVector3(b).sub(a);
  float l = d.lengthSquared();
  if (l > GJK_SIMPLEX2_EPS) {
   float t = (l > 0 ? -btDot(a, d) / l : 0);
   if (t >= 1) {
    w[0] = 0;
    w[1] = 1;
    m[0] = 2;
    return (b.lengthSquared());
   } else if (t <= 0) {
    w[0] = 1;
    w[1] = 0;
    m[0] = 1;
    return (a.lengthSquared());
   } else {
    w[0] = 1 - (w[1] = t);
    m[0] = 3;
    return (new btVector3().scaleAdd(t, d, a).lengthSquared());
   }
  }
  return (-1);
 }

 static float projectorigin(final btVector3 a, final btVector3 b,
  final btVector3 c, float[] w,
  int[] m) {
  btVector3[] vt = new btVector3[]{a, b, c};
  btVector3[] dl = new btVector3[]{new btVector3(a).sub(b), new btVector3(b)
   .sub(c),
   new btVector3(c).sub(a)};
  final btVector3 n = new btVector3(dl[0]).cross(dl[1]);
  float l = n.lengthSquared();
  if (l > GJK_SIMPLEX3_EPS) {
   float mindist = -1;
   float[] subw = new float[]{0.f, 0.f};
   int[] subm = new int[1];
   for (int i = 0; i < 3; ++i) {
    if (btDot(vt[i], new btVector3(dl[i]).cross(n)) > 0) {
     int j = imd3[i];
     float subd = projectorigin(vt[i], vt[j], subw, subm);
     if ((mindist < 0) || (subd < mindist)) {
      mindist = subd;
      m[0] = (((subm[0] & 1) != 0 ? 1 << i : 0)
       + ((subm[0] & 2) != 0 ? 1 << j : 0));
      w[i] = subw[0];
      w[j] = subw[1];
      w[imd3[j]] = 0;
     }
    }
   }
   if (mindist < 0) {
    float d = btDot(a, n);
    float s = btSqrt(l);
    final btVector3 p = new btVector3(n).scale(d / l);
    mindist = p.lengthSquared();
    m[0] = 7;
    w[0] = (new btVector3(dl[1]).cross(new btVector3(b).sub(p))).length() / s;
    w[1] = (new btVector3(dl[2]).cross(new btVector3(c).sub(p))).length() / s;
    w[2] = 1 - (w[0] + w[1]);
   }
   return (mindist);
  }
  return (-1);
 }

 static float projectorigin(final btVector3 a, final btVector3 b,
  final btVector3 c,
  final btVector3 d,
  float[] w, int[] m) {
  btVector3 vt[] = {a, b, c, d};
  btVector3 dl[] = {new btVector3(a).sub(d), new btVector3(b).sub(d),
   new btVector3(c).sub(d)};
  float vl = det(dl[0], dl[1], dl[2]);
  boolean ng = (vl * btDot(a, new btVector3(b).sub(c).cross(new btVector3(a)
   .sub(b)))) <= 0;
  if (ng && (btFabs(vl) > GJK_SIMPLEX4_EPS)) {
   float mindist = -1;
   float[] subw = new float[3];
   int[] subm = new int[1];
   for (int i = 0; i < 3; ++i) {
    int j = imd3[i];
    float s = vl * btDot(d, new btVector3(dl[i]).cross(dl[j]));
    if (s > 0) {
     float subd = projectorigin(vt[i], vt[j], d, subw, subm);
     if ((mindist < 0) || (subd < mindist)) {
      mindist = subd;
      m[0] = (((subm[0] & 1) != 0 ? 1 << i : 0)
       + ((subm[0] & 2) != 0 ? 1 << j : 0) + ((subm[0] & 4) != 0 ? 8 : 0));
      w[i] = subw[0];
      w[j] = subw[1];
      w[imd3[j]] = 0;
      w[3] = subw[2];
     }
    }
   }
   if (mindist < 0) {
    mindist = 0;
    m[0] = 15;
    w[0] = det(c, b, d) / vl;
    w[1] = det(a, c, d) / vl;
    w[2] = det(b, a, d) / vl;
    w[3] = 1 - (w[0] + w[1] + w[2]);
   }
   return (mindist);
  }
  return (-1);
 }

};
