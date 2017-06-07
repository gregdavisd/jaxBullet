/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision.Shape;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE;
import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
abstract public class btCollisionShape implements Serializable {

 protected int m_shapeType;
 protected Object m_userPointer;
 protected int m_userIndex;

 public btCollisionShape() {
  m_shapeType = INVALID_SHAPE_PROXYTYPE;
  m_userPointer = null;
  m_userIndex = (-1);
 }

 ///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
 public abstract void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax);

 /**
  *
  * @param center output
  * @return radius
  */
 public float getBoundingSphere(final btVector3 center) {
  final btTransform tr = new btTransform();
  tr.setIdentity();
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  getAabb(tr, aabbMin, aabbMax);
  float radius = new btVector3(aabbMax).sub(aabbMin).length() * 0.5f;
  center.set(new btVector3(aabbMin).add(aabbMax).scale(0.5f));
  return radius;
 }

 ///getAngularMotionDisc returns the maximum radius needed for Conservative Advancement to handle time-of-impact with rotations.
 public float getAngularMotionDisc() {
  ///@todo cache this value, to improve performance
  final btVector3 center = new btVector3();
  float disc;
  disc = getBoundingSphere(center);
  disc += (center).length();
  return disc;
 }

 public float getContactBreakingThreshold(float defaultContactThresholdFactor) {
  return getAngularMotionDisc() * defaultContactThresholdFactor;
 }

 ///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
 ///result is conservative
 public void calculateTemporalAabb(final btTransform curTrans, final btVector3 linvel,
  final btVector3 angvel,
  float timeStep, final btVector3 temporalAabbMin, final btVector3 temporalAabbMax) {
  //start with static aabb
  getAabb(curTrans, temporalAabbMin, temporalAabbMax);
  float temporalAabbMaxx = temporalAabbMax.getX();
  float temporalAabbMaxy = temporalAabbMax.getY();
  float temporalAabbMaxz = temporalAabbMax.getZ();
  float temporalAabbMinx = temporalAabbMin.getX();
  float temporalAabbMiny = temporalAabbMin.getY();
  float temporalAabbMinz = temporalAabbMin.getZ();
  // add linear motion
  final btVector3 linMotion = new btVector3(linvel).scale(timeStep);
  ///@todo: simd would have a vector max/min operation, instead of per-element access
  if (linMotion.x > 0f) {
   temporalAabbMaxx += linMotion.x;
  } else {
   temporalAabbMinx += linMotion.x;
  }
  if (linMotion.y > 0f) {
   temporalAabbMaxy += linMotion.y;
  } else {
   temporalAabbMiny += linMotion.y;
  }
  if (linMotion.z > 0f) {
   temporalAabbMaxz += linMotion.z;
  } else {
   temporalAabbMinz += linMotion.z;
  }
  //add conservative angular motion
  float angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
  final btVector3 angularMotion3d = new btVector3(angularMotion, angularMotion, angularMotion);
  temporalAabbMin.set(new btVector3(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz));
  temporalAabbMax.set(new btVector3(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz));
  temporalAabbMin.sub(angularMotion3d);
  temporalAabbMax.add(angularMotion3d);
 }

 public boolean isPolyhedral() {
  return btBroadphaseProxy.isPolyhedral(getShapeType());
 }

 public boolean isConvex2d() {
  return btBroadphaseProxy.isConvex2d(getShapeType());
 }

 public boolean isConvex() {
  return btBroadphaseProxy.isConvex(getShapeType());
 }

 public boolean isNonMoving() {
  return btBroadphaseProxy.isNonMoving(getShapeType());
 }

 public boolean isConcave() {
  return btBroadphaseProxy.isConcave(getShapeType());
 }

 public boolean isCompound() {
  return btBroadphaseProxy.isCompound(getShapeType());
 }

 public boolean isSoftBody() {
  return btBroadphaseProxy.isSoftBody(getShapeType());
 }

 ///isInfinite is used to catch simulation error (aabb check)
 public boolean isInfinite() {
  return btBroadphaseProxy.isInfinite(getShapeType());
 }

 public abstract void setLocalScaling(final btVector3 scaling);

 public abstract btVector3 getLocalScaling();

 public abstract void calculateLocalInertia(float mass, final btVector3 inertia);

//debugging support
 public abstract String getName();

 public int getShapeType() {
  return m_shapeType;
 }

 ///the getAnisotropicRollingFrictionDirection can be used in combination with setAnisotropicFriction
 ///See Bullet/Demos/RollingFrictionDemo for an example
 public btVector3 getAnisotropicRollingFrictionDirection() {
  return new btVector3(1, 1, 1);
 }

 public abstract void setMargin(float margin);

 public abstract float getMargin();

 ///optional user data pointer
 public void setUserPointer(Object userPtr) {
  m_userPointer = userPtr;
 }

 public Object getUserPointer() {
  return m_userPointer;
 }

 public void setUserIndex(int index) {
  m_userIndex = index;
 }

 public int getUserIndex() {
  return m_userIndex;
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash = 79 * hash + this.m_shapeType;
  return hash;
 }

 @Override
 public boolean equals(Object obj) {
  if (this == obj) {
   return true;
  }
  if (obj == null) {
   return false;
  }
  if (getClass() != obj.getClass()) {
   return false;
  }
  final btCollisionShape other = (btCollisionShape) obj;
  return this.m_shapeType == other.m_shapeType;
 }
}
