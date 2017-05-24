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

import static Bullet.LinearMath.btAabbUtil2.btTransformAabb;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btPolyhedralConvexAabbCachingShape extends btPolyhedralConvexShape  implements Serializable {

 final btVector3 m_localAabbMin = new btVector3();
 final btVector3 m_localAabbMax = new btVector3();
 boolean m_isLocalAabbValid;

 void setCachedLocalAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  m_isLocalAabbValid = true;
  m_localAabbMin.set(aabbMin);
  m_localAabbMax.set(aabbMax);
 }

 void getCachedLocalAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
  assert(m_isLocalAabbValid);
  aabbMin.set(m_localAabbMin);
  aabbMax.set(m_localAabbMax);
 }

 btPolyhedralConvexAabbCachingShape() {
  super();
  m_localAabbMin.set(1, 1, 1);
  m_localAabbMax.set(-1, -1, -1);
  m_isLocalAabbValid = false;
 }

 void getNonvirtualAabb(final btTransform trans, final btVector3 aabbMin, final btVector3 aabbMax,
  float margin) {
  //lazy evaluation of local aabb
  assert(m_isLocalAabbValid);
  btTransformAabb(m_localAabbMin, m_localAabbMax, margin, trans, aabbMin, aabbMax);
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  super.setLocalScaling(scaling);
  recalcLocalAabb();
 }

 @Override
 public void getAabb(final btTransform trans, final btVector3 aabbMin, final btVector3 aabbMax) {
  getNonvirtualAabb(trans, aabbMin, aabbMax, getMargin());
 }
 static btVector3[] _directions = new btVector3[]{
  new btVector3(1.f, 0.f, 0.f),
  new btVector3(0.f, 1.f, 0.f),
  new btVector3(0.f, 0.f, 1.f),
  new btVector3(-1.f, 0.f, 0.f),
  new btVector3(0.f, -1.f, 0.f),
  new btVector3(0.f, 0.f, -1.f)
 };

 final void recalcLocalAabb() {
  m_isLocalAabbValid = true;
  btVector3[] _supporting = new btVector3[]{
   new btVector3(0.f, 0.f, 0.f),
   new btVector3(0.f, 0.f, 0.f),
   new btVector3(0.f, 0.f, 0.f),
   new btVector3(0.f, 0.f, 0.f),
   new btVector3(0.f, 0.f, 0.f),
   new btVector3(0.f, 0.f, 0.f)
  };
  batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
  for (int i = 0; i < 3; ++i) {
   m_localAabbMax.setElement(i, _supporting[i].getElement(i) + m_collisionMargin);
   m_localAabbMin.setElement(i, _supporting[i + 3].getElement(i) - m_collisionMargin);
  }
 }
}
