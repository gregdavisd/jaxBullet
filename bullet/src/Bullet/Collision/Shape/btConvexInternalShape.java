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

import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btConvexInternalShape extends btConvexShape implements Serializable {

 public static final float CONVEX_DISTANCE_MARGIN = 0.04f;
 //local scaling. collisionMargin is not scaled !
 final btVector3 m_localScaling;
 final btVector3 m_implicitShapeDimensions;
 float m_collisionMargin;
 float m_padding;

 btConvexInternalShape() {
  m_localScaling = new btVector3((1.f), (1.f), (1.f));
  m_collisionMargin = CONVEX_DISTANCE_MARGIN;
  m_implicitShapeDimensions = new btVector3();
 }

 @Override
 public btVector3 localGetSupportingVertex(final btVector3 vec) {
  final btVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);
  if (getMargin() != (0.f)) {
   final btVector3 vecnorm = new btVector3(vec);
   if (vecnorm.lengthSquared() < (SIMD_EPSILON * SIMD_EPSILON)) {
    vecnorm.set((-1.f), (-1.f), (-1.f));
   }
   vecnorm.normalize();
   supVertex.add(vecnorm.scale(getMargin()));
  }
  return supVertex;
 }

 btVector3 getImplicitShapeDimensions() {
  return new btVector3(m_implicitShapeDimensions);
 }

 ///warning: use setImplicitShapeDimensions with care
 ///changing a collision shape while the body is in the world is not recommended,
 ///it is best to remove the body from the world, then make the change, and re-add it
 ///alternatively flush the contact points, see documentation for 'cleanProxyFromPairs'
 void setImplicitShapeDimensions(final btVector3 dimensions) {
  m_implicitShapeDimensions.set(dimensions);
 }

 void setSafeMargin(float minDimension) {
  setSafeMargin(minDimension, 0.1f);
 }

 void setSafeMargin(float minDimension, float defaultMarginMultiplier) {
  float safeMargin = defaultMarginMultiplier * minDimension;
  if (safeMargin < getMargin()) {
   setMargin(safeMargin);
  }
 }

 final void setSafeMargin(final btVector3 halfExtents) {
  setSafeMargin(halfExtents, 0.1f);
 }

 void setSafeMargin(final btVector3 halfExtents, float defaultMarginMultiplier) {
  //see http://code.google.com/p/bullet/issues/detail?id=349
  //this margin check could could be added to other collision shapes too,
  //or add some assert/warning somewhere
  float minDimension = halfExtents.min();
  setSafeMargin(minDimension, defaultMarginMultiplier);
 }

 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 @Override
 public void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax) {
  getAabbSlow(t, aabbMin, aabbMax);
 }

 @Override
 public void getAabbSlow(final btTransform trans, final btVector3 aabbMin, final btVector3 aabbMax) {
  //use localGetSupportingVertexWithoutMargin?
  float margin = getMargin();
  for (int i = 0; i < 3; i++) {
   final btVector3 vec = new btVector3();
   vec.setElement(i, 1.f);
   final btVector3 sv = localGetSupportingVertex(trans.transposeTransform3x3(new btVector3(vec)));
   final btVector3 tmp = trans.transform(new btVector3(sv));
   aabbMax.setElement(i, tmp.getElement(i) + margin);
   vec.setElement(i, -1.f);
   tmp.set(trans
    .transform(localGetSupportingVertex(trans.transposeTransform3x3(new btVector3(vec)))));
   aabbMin.setElement(i, tmp.getElement(i) - margin);
  }
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  m_localScaling.set(scaling).abs();
 }

 @Override
 public btVector3 getLocalScaling() {
  return new btVector3(m_localScaling);
 }

 public btVector3 getLocalScalingNV() {
  return new btVector3(m_localScaling);
 }

 @Override
 public void setMargin(float margin) {
  m_collisionMargin = margin;
 }

 @Override
 public float getMargin() {
  return m_collisionMargin;
 }

 public final float getMarginNV() {
  return m_collisionMargin;
 }

 @Override
 public int getNumPreferredPenetrationDirections() {
  return 0;
 }

 @Override
 public void getPreferredPenetrationDirection(int index, final btVector3 penetrationVector) {
  assert (false);
 }
};
