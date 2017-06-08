/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics.CollisionObjects;

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btHashedOverlappingPairCache;
import Bullet.Collision.btCollisionObject;
import static Bullet.common.btAlignedObjectArray.findLinearSearch;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btPairCachingGhostObject extends btGhostObject implements Serializable {

 private final btHashedOverlappingPairCache m_hashPairCache;

 public btPairCachingGhostObject() {
  m_hashPairCache = new btHashedOverlappingPairCache();
 }

 ///this method is mainly for expert/internal use only.
 @Override
 public void addOverlappingObjectInternal(btBroadphaseProxy otherProxy, btBroadphaseProxy thisProxy) {
  btBroadphaseProxy actualThisProxy = thisProxy != null ? thisProxy : getBroadphaseHandle();
  assert (actualThisProxy != null);
  btCollisionObject otherObject = (btCollisionObject) otherProxy.m_clientObject;
  assert (otherObject != null);
  int index = findLinearSearch(m_overlappingObjects, otherObject);
  if (index == m_overlappingObjects.size()) {
   m_overlappingObjects.add(otherObject);
   m_hashPairCache.addOverlappingPair(actualThisProxy, otherProxy);
  }
 }

 @Override
 public void removeOverlappingObjectInternal(btBroadphaseProxy otherProxy, btDispatcher dispatcher,
  btBroadphaseProxy thisProxy1) {
  btCollisionObject otherObject = (btCollisionObject) otherProxy.m_clientObject;
  btBroadphaseProxy actualThisProxy = thisProxy1 != null ? thisProxy1 : getBroadphaseHandle();
  assert (actualThisProxy != null);
  assert (otherObject != null);
  int index = findLinearSearch(m_overlappingObjects, otherObject);
  if (index < m_overlappingObjects.size()) {
   m_overlappingObjects.set(index, m_overlappingObjects.get(m_overlappingObjects.size() - 1));
   m_overlappingObjects.remove(m_overlappingObjects.size() - 1);
   m_hashPairCache.removeOverlappingPair(actualThisProxy, otherProxy, dispatcher);
  }
 }

 public btHashedOverlappingPairCache getOverlappingPairCache() {
  return m_hashPairCache;
 }
}
