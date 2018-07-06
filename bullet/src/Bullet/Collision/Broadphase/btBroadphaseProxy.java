/*
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
package Bullet.Collision.Broadphase;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.BOX_2D_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONCAVE_SHAPES_END_HERE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONCAVE_SHAPES_START_HERE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.CONVEX_2D_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.GIMPACT_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.IMPLICIT_CONVEX_SHAPES_START_HERE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.SOFTBODY_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btBroadphaseProxy implements Serializable {

 public static final int DEFAULT_FILTER = 1;
 public static final int STATIC_FILTER = 2;
 public static final int KINEMATIC_FILTER = 4;
 public static final int DEBRIS_FILTER = 8;
 public static final int SENSOR_TRIGGER = 16;
 public static final int CHARACTER_FILTER = 32;
 public static final int ALL_FILTER = -1; //all bits sets: DefaultFilter | StaticFilter | KinematicFilter | DebrisFilter | SensorTrigger

 public static boolean isPolyhedral(int proxyType) {
  return (proxyType < IMPLICIT_CONVEX_SHAPES_START_HERE);
 }

 public static boolean isConvex(int proxyType) {
  return (proxyType < CONCAVE_SHAPES_START_HERE);
 }

 public static boolean isNonMoving(int proxyType) {
  return (isConcave(proxyType) && !(proxyType == GIMPACT_SHAPE_PROXYTYPE));
 }

 public static boolean isConcave(int proxyType) {
  return ((proxyType > CONCAVE_SHAPES_START_HERE) && (proxyType
   < CONCAVE_SHAPES_END_HERE));
 }

 public static boolean isCompound(int proxyType) {
  return (proxyType == COMPOUND_SHAPE_PROXYTYPE);
 }

 public static boolean isSoftBody(int proxyType) {
  return (proxyType == SOFTBODY_SHAPE_PROXYTYPE);
 }

 public static boolean isInfinite(int proxyType) {
  return (proxyType == STATIC_PLANE_PROXYTYPE);
 }

 public static boolean isConvex2d(int proxyType) {
  return (proxyType == BOX_2D_SHAPE_PROXYTYPE) || (proxyType
   == CONVEX_2D_SHAPE_PROXYTYPE);
 }

 //Usually the client btCollisionObject or Rigidbody class
 public Object m_clientObject;
 public int m_collisionFilterGroup;
 public int m_collisionFilterMask;
 int m_uniqueId;//m_uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.
 public final btVector3 m_aabbMin = new btVector3();
 public final btVector3 m_aabbMax = new btVector3();

 //used for memory pools
 public btBroadphaseProxy() {
  m_clientObject = null;
 }

 public btBroadphaseProxy(final btVector3 aabbMin, final btVector3 aabbMax,
  Object userPtr,
  int collisionFilterGroup,
  int collisionFilterMask) {
  m_clientObject = userPtr;
  m_collisionFilterGroup = collisionFilterGroup;
  m_collisionFilterMask = collisionFilterMask;
  m_aabbMin.set(aabbMin);
  m_aabbMax.set(aabbMax);
 }

 public int getUid() {
  return m_uniqueId;
 }

 public abstract boolean intersect(btBroadphaseProxy other);

}
