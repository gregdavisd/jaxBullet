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

import static Bullet.Collision.GJK.GJK_MIN_DISTANCE;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_INFINITY;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btGjkEpaSolver2 implements Serializable {

 static int StackSizeRequirement() {
  // no idea what for
  throw new AssertionError();
  //return(sizeof(GJK)+sizeof(EPA));
 }

 static boolean Distance(btConvexShape shape0, final btTransform wtrs0,
  btConvexShape shape1, final btTransform wtrs1, final btVector3 guess,
  sResults results) {
  MinkowskiDiff shape = new MinkowskiDiff();
  Initialize(shape0, wtrs0, shape1, wtrs1, results, shape, false);
  GJK gjk = new GJK();
  GJK.eStatus gjk_status = gjk.evaluate(shape, guess);
  if (gjk_status == GJK.eStatus.Valid) {
   final btVector3 w0 = new btVector3();
   final btVector3 w1 = new btVector3();
   for (int i = 0; i < gjk.m_simplex.rank; ++i) {
    float p = gjk.m_simplex.p[i];
    w0.add(shape.support(gjk.m_simplex.c[i].d, 0).scale(p));
    w1.add(shape.support(new btVector3(gjk.m_simplex.c[i].d).negate(), 1).scale(
     p));
   }
   results.witnesses[0].set(wtrs0.transform(new btVector3(w0)));
   results.witnesses[1].set(wtrs0.transform(new btVector3(w1)));
   results.normal.set(w0).sub(w1);
   results.distance = results.normal.length();
   results.normal.scale(1.0f
    / (results.distance > GJK_MIN_DISTANCE ? results.distance : 1.0f));
   return (true);
  } else {
   results.status
    = (gjk_status == GJK.eStatus.Inside)
     ? sResults.eStatus.Penetrating
     : sResults.eStatus.GJK_Failed;
   return (false);
  }
 }

 static boolean Penetration(btConvexShape shape0, final btTransform wtrs0,
  btConvexShape shape1, final btTransform wtrs1, final btVector3 guess,
  sResults results) {
  return Penetration(shape0, wtrs0, shape1, wtrs1, guess, results, true);
 }

 static boolean Penetration(btConvexShape shape0, final btTransform wtrs0,
  btConvexShape shape1, final btTransform wtrs1, final btVector3 guess,
  sResults results,
  boolean usemargins) {
  MinkowskiDiff shape = new MinkowskiDiff();
  Initialize(shape0, wtrs0, shape1, wtrs1, results, shape, usemargins);
  GJK gjk = new GJK();
  GJK.eStatus gjk_status = gjk.evaluate(shape, new btVector3(guess).negate());
  switch (gjk_status) {
   case Inside: {
    EPA epa = new EPA();
    EPA.eStatus epa_status = epa.evaluate(gjk, new btVector3(guess).negate());
    if (epa_status != EPA.eStatus.Failed) {
     final btVector3 w0 = new btVector3();
     for (int i = 0; i < epa.m_result.rank; ++i) {
      w0.add(shape.support(epa.m_result.c[i].d, 0).scale(epa.m_result.p[i]));
     }
     results.status = sResults.eStatus.Penetrating;
     wtrs0.transform(results.witnesses[0].set(w0));
     wtrs0.transform(results.witnesses[1].set(w0)
      .sub(new btVector3(epa.m_normal).scale(epa.m_depth)));
     results.normal.set(epa.m_normal).negate();
     results.distance = -epa.m_depth;
     return (true);
    } else {
     results.status = sResults.eStatus.EPA_Failed;
    }
   }
   break;
   case Failed:
    results.status = sResults.eStatus.GJK_Failed;
    break;
   default: {
   }
  }
  return (false);
 }

 static float SignedDistance(final btVector3 position,
  float margin,
  btConvexShape shape0, final btTransform wtrs0,
  sResults results) {
  MinkowskiDiff shape = new MinkowskiDiff();
  btSphereShape shape1 = new btSphereShape(margin);
  final btTransform wtrs1 = new btTransform(new btQuaternion(0f, 0f, 0f, 1f),
   position);
  Initialize(shape0, wtrs0, shape1, wtrs1, results, shape, false);
  GJK gjk = new GJK();
  GJK.eStatus gjk_status = gjk.evaluate(shape, new btVector3(1, 1, 1));
  if (gjk_status == GJK.eStatus.Valid) {
   final btVector3 w0 = new btVector3();
   final btVector3 w1 = new btVector3();
   for (int i = 0; i < gjk.m_simplex.rank; ++i) {
    float p = gjk.m_simplex.p[i];
    w0.add(shape.support(gjk.m_simplex.c[i].d, 0).scale(p));
    w1.add(shape.support(new btVector3(gjk.m_simplex.c[i].d).negate(), 1).scale(
     p));
   }
   wtrs0.transform(results.witnesses[0].set(w0));
   wtrs0.transform(results.witnesses[1].set(w1));
   final btVector3 delta = new btVector3(results.witnesses[1]).sub(
    results.witnesses[0]);
   float margin_2 = shape0.getMarginNonVirtual() + shape1.getMarginNonVirtual();
   float length = delta.length();
   results.normal.set(delta).scale(1.0f / length);
   results.witnesses[0].add(new btVector3(results.normal).scale(margin));
   return (length - margin_2);
  } else if (gjk_status == GJK.eStatus.Inside) {
   if (Penetration(shape0, wtrs0, shape1, wtrs1, gjk.m_ray, results)) {
    final btVector3 delta = new btVector3(results.witnesses[0]).sub(
     results.witnesses[1]);
    float length = delta.length();
    if (length >= SIMD_EPSILON) {
     results.normal.set(delta).scale(1.0f / length);
    }
    return (-length);
   }
  }
  return (SIMD_INFINITY);
 }

 static boolean SignedDistance(btConvexShape shape0, final btTransform wtrs0,
  btConvexShape shape1, final btTransform wtrs1, final btVector3 guess,
  sResults results) {
  if (!Distance(shape0, wtrs0, shape1, wtrs1, guess, results)) {
   return (Penetration(shape0, wtrs0, shape1, wtrs1, guess, results, false));
  } else {
   return (true);
  }
 }

 static void Initialize(btConvexShape shape0, final btTransform wtrs0,
  btConvexShape shape1, final btTransform wtrs1,
  btGjkEpaSolver2.sResults results,
  MinkowskiDiff shape,
  boolean withmargins) {
  /*
   * Results
   */
  results.witnesses[0].setZero();
  results.witnesses[1].setZero();
  results.status = btGjkEpaSolver2.sResults.eStatus.Separated;
  /*
   * Shape
   */
  shape.m_shapes[0] = shape0;
  shape.m_shapes[1] = shape1;
  shape.m_toshape1.set(wtrs1.getBasis().transposeTimes(wtrs0.getBasis()));
  shape.m_toshape0.set(wtrs0.inverseTimes(wtrs1));
  shape.enableMargin(withmargins);
 }

 static class sResults {

  static enum eStatus {
   Separated, /*
    * Shapes doesnt penetrate
    */
   Penetrating, /*
    * Shapes are penetrating
    */
   GJK_Failed, /*
    * GJK phase fail, no big issue, shapes are probably just 'touching'
    */
   EPA_Failed
   /*
    * EPA phase fail, bigger problem, need to save parameters, and debug
    */
  };
  eStatus status;
  final btVector3[] witnesses = new btVector3[2];
  final btVector3 normal = new btVector3();
  float distance;

  public sResults() {
   init(witnesses);
  }

 }
}
