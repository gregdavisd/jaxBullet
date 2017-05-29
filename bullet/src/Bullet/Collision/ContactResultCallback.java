/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

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
///ContactResultCallback is used to report contact points

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
abstract public class ContactResultCallback implements Serializable {

 int m_collisionFilterGroup;
 int m_collisionFilterMask;
 float m_closestDistanceThreshold;

 public ContactResultCallback() {
  m_collisionFilterGroup = btBroadphaseProxy.DefaultFilter;
  m_collisionFilterMask = btBroadphaseProxy.AllFilter;
  m_closestDistanceThreshold = 0;
 }

 public boolean needsCollision(btBroadphaseProxy proxy0
 ) {
  boolean collides = (proxy0.m_collisionFilterGroup & m_collisionFilterMask) != 0;
  collides = collides && ((m_collisionFilterGroup & proxy0.m_collisionFilterMask) != 0);
  return collides;
 }

 public abstract float addSingleResult(btManifoldPoint cp, btCollisionObjectWrapper colObj0Wrap,
  int partId0,
  int index0, btCollisionObjectWrapper colObj1Wrap,
  int partId1,
  int index1
 );
};
