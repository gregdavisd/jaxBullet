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
package Bullet.Collision;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btBridgedManifoldResult extends btManifoldResult implements Serializable {

 final ContactResultCallback m_resultCallback;

 public btBridgedManifoldResult(btCollisionObjectWrapper obj0Wrap, btCollisionObjectWrapper obj1Wrap,
  ContactResultCallback resultCallback) {
  super(obj0Wrap, obj1Wrap);
  m_resultCallback = resultCallback;
 }

 @Override
 public void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorld,
  float depth) {
  boolean isSwapped = m_manifoldPtr.getBody0() != m_body0Wrap.getCollisionObject();
  final btVector3 pointA = new btVector3(pointInWorld).add(new btVector3(normalOnBInWorld).scale(
   depth));
  final btVector3 localA;
  final btVector3 localB;
  if (isSwapped) {
   localA = m_body1Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointA);
   localB = m_body0Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointInWorld);
  } else {
   localA = m_body0Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointA);
   localB = m_body1Wrap.getCollisionObject().getWorldTransformPtr().invXform(pointInWorld);
  }
  btManifoldPoint newPt = new btManifoldPoint(localA, localB, normalOnBInWorld, depth);
  newPt.m_positionWorldOnA.set(pointA);
  newPt.m_positionWorldOnB.set(pointInWorld);
  //BP mod, store contact triangles.
  if (isSwapped) {
   newPt.m_partId0 = m_partId1;
   newPt.m_partId1 = m_partId0;
   newPt.m_index0 = m_index1;
   newPt.m_index1 = m_index0;
  } else {
   newPt.m_partId0 = m_partId0;
   newPt.m_partId1 = m_partId1;
   newPt.m_index0 = m_index0;
   newPt.m_index1 = m_index1;
  }
  //experimental feature info, for per-triangle material etc.
  btCollisionObjectWrapper obj0Wrap = isSwapped ? m_body1Wrap : m_body0Wrap;
  btCollisionObjectWrapper obj1Wrap = isSwapped ? m_body0Wrap : m_body1Wrap;
  m_resultCallback.addSingleResult(newPt, obj0Wrap, newPt.m_partId0, newPt.m_index0, obj1Wrap,
   newPt.m_partId1, newPt.m_index1);
 }
};
