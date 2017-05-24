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

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collection;

/**
 *
 * @author Gregery Barton
 */
abstract public class btOverlappingPairCache implements btOverlappingPairCallback , Serializable  {

 static final int BT_NULL_PAIR = 0xffffffff;

 public abstract ArrayList<btBroadphasePair> getOverlappingPairArrayPtr();

 public abstract Collection<btBroadphasePair> getOverlappingPairArray();

 public abstract void cleanOverlappingPair(btBroadphasePair pair, btDispatcher dispatcher);

 public abstract int getNumOverlappingPairs();

 public abstract void cleanProxyFromPairs(btBroadphaseProxy proxy, btDispatcher dispatcher);

 public abstract void setOverlapFilterCallback(btOverlapFilterCallback callback);

 public abstract void processAllOverlappingPairs(btOverlapCallback callback, btDispatcher dispatcher);

 public abstract btBroadphasePair findPair(btBroadphaseProxy proxy0, btBroadphaseProxy proxy1);

 public abstract boolean hasDeferredRemoval();

 public abstract void setInternalGhostPairCallback(btOverlappingPairCallback ghostPairCallback);

 public abstract void sortOverlappingPairs(btDispatcher dispatcher);

 public abstract void incrementalCleanup(int ni, btDispatcher dispatcher);
};
