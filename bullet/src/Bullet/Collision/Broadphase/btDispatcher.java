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
package Bullet.Collision.Broadphase;
///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).

import Bullet.Collision.Algorithm.btCollisionAlgorithm;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btPersistentManifold;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public interface btDispatcher  extends Serializable {

 /**
  *
  * @param body0Wrap
  * @param body1Wrap
  * @param sharedManifold
  * @param queryType
  * @return
  */
 public btCollisionAlgorithm findAlgorithm(
  btCollisionObjectWrapper body0Wrap,
  btCollisionObjectWrapper body1Wrap,
  btPersistentManifold sharedManifold,
  int queryType);

 /**
  *
  * @param b0
  * @param b1
  * @return
  */
 public btPersistentManifold getNewManifold(btCollisionObject b0, btCollisionObject b1);

 /**
  *
  * @param manifold
  */
 public void releaseManifold(btPersistentManifold manifold);

 /**
  *
  * @param manifold
  */
 public void clearManifold(btPersistentManifold manifold);

 /**
  *
  * @param body0
  * @param body1
  * @return
  */
 public boolean needsCollision(btCollisionObject body0, btCollisionObject body1);

 /**
  *
  * @param body0
  * @param body1
  * @return
  */
 public boolean needsResponse(btCollisionObject body0, btCollisionObject body1);

 /**
  *
  * @param pairCache
  * @param dispatchInfo
  * @param dispatcher
  */
 public void dispatchAllCollisionPairs(btOverlappingPairCache pairCache,
  btDispatcherInfo dispatchInfo,
  btDispatcher dispatcher);

 /**
  *
  * @return
  */
 public int getNumManifolds();

 /**
  *
  * @param index
  * @return
  */
 public btPersistentManifold getManifoldByIndexInternal(int index);

 ArrayList<btPersistentManifold> getInternalManifoldPointer();
}
