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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Objects;

///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
///It is persistent over frames
/**
 *
 * @author Gregery Barton
 */
public abstract class btCollisionAlgorithm implements Serializable {

 private static final long serialVersionUID = 1L;

 @Override
 public int hashCode() {
  int hash = 3;
  hash = 23 * hash + Objects.hashCode(this.m_dispatcher);
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
  final btCollisionAlgorithm other = (btCollisionAlgorithm) obj;
  if (!Objects.equals(this.m_dispatcher, other.m_dispatcher)) {
   return false;
  }
  return true;
 }

 final btDispatcher m_dispatcher;

 public btCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci) {
  m_dispatcher = ci.m_dispatcher1;
 }

 public abstract void processCollision(btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap, btDispatcherInfo dispatchInfo,
  btManifoldResult resultOut);

 public abstract float calculateTimeOfImpact(btCollisionObject body0,
  btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut);

 public abstract void getAllContactManifolds(
  ArrayList<btPersistentManifold> manifoldArray);

 public abstract void destroy();

}
