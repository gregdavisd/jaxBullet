/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.LinearMath;

import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.SIMD_HALF_PI;
import static Bullet.LinearMath.btScalar.btCos;
import static Bullet.LinearMath.btScalar.btSin;
import static Bullet.LinearMath.btScalar.btSqrt;
import java.io.Serializable;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import static javax.vecmath.VecMath.is_good_matrix;

/**
 *
 * @author Gregery Barton
 */
public class btTransformUtil implements Serializable {

 static final float ANGULAR_MOTION_THRESHOLD = 0.5f * SIMD_HALF_PI;

 /**
  *
  * @param transform0
  * @param transform1
  * @param timeStep
  * @param linVel
  * @param angVel
  */
 public static void calculateVelocity(final btTransform transform0, final btTransform transform1,
  float timeStep, final btVector3 linVel, final btVector3 angVel) {
  linVel.set(transform1.getOrigin()).sub(transform0.getOrigin()).scale(1.0f / timeStep);
  float angle = calculateDiffAxisAngle(transform0, transform1, angVel);
  angVel.scale(angle / timeStep);
  if (DEBUG_BLOCKS) {
  }
 }

 /**
  *
  * @param transform0
  * @param transform1
  * @param axis
  * @return
  */
 public static float calculateDiffAxisAngle(final btTransform transform0,
  final btTransform transform1, final btVector3 axis) {
  final btMatrix3x3 dmat = transform1.getBasis().mul(transform0.getBasis().invert());
  final btQuaternion dorn = new btQuaternion().set(dmat);
  ///floating point inaccuracy can lead to w component > 1..., which breaks 
  //dorn.normalize();
  float angle = dorn.getAngle();
  axis.set(new btVector3(dorn.x(), dorn.y(), dorn.z()));
  //axis.w = 0f;
  //check for axis length
  float len = axis.lengthSquared();
  if (len < SIMD_EPSILON * SIMD_EPSILON) {
   axis.set(1, 0, 0);
  } else {
   axis.scale(1.0f / btSqrt(len));
  }
  return angle;
 }

 /**
  *
  * @param curTrans
  * @param linvel
  * @param angvel
  * @param timeStep
  * @param predictedTransform
  */
 public static void integrateTransform(final btTransform curTrans, final btVector3 linvel,
  final btVector3 angvel,
  float timeStep, final btTransform predictedTransform) {
  assert (is_good_matrix(curTrans));
//  assert (timeStep >= 0);
  final btVector3 predictedOrigin = new btVector3().scaleAdd(timeStep, linvel, curTrans.getOrigin());
  //Exponential map
  //google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia
  final btVector3 axis = new btVector3();
  float fAngle = angvel.length();
  assert (Float.isFinite(fAngle));
  //limit the angular motion
  if (fAngle * timeStep > ANGULAR_MOTION_THRESHOLD) {
   fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
  }
  assert (Float.isFinite(fAngle));
  if (fAngle < (0.001f)) {
   // use Taylor's expansions of sync function
   axis.set(angvel).scale((0.5f * timeStep) - (timeStep * timeStep * timeStep) *
    ((0.020833333333f)) * fAngle * fAngle);
  } else {
   // sync(fAngle) = sin(c*fAngle)/t
   axis.set(angvel).scale(btSin(0.5f * fAngle * timeStep) / fAngle);
  }
  final btQuaternion dorn =
   new btQuaternion(axis.x(), axis.y(), axis.z(), btCos(fAngle * timeStep * 0.5f));
  final btQuaternion orn0 = curTrans.getRotation();
  final btQuaternion predictedOrn = new btQuaternion(dorn).mul(orn0);
  predictedOrn.normalize();
  predictedTransform.set(predictedOrn, predictedOrigin);
  assert (is_good_matrix(predictedTransform));
 }
}
