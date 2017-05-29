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

/*
 
  @author Gregery Barton
 */
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
abstract public class btBroadphaseInterface implements Serializable {

 public abstract btBroadphaseProxy createProxy(final btVector3 aabbMin, final btVector3 aabbMax,
  int shapeType,
  Object userPtr, int collisionFilterGroup, int collisionFilterMask, btDispatcher dispatcher);

 public abstract void destroyProxy(btBroadphaseProxy proxy, btDispatcher dispatcher);

 public abstract void setAabb(btBroadphaseProxy proxy, final btVector3 aabbMin,
  final btVector3 aabbMax,
  btDispatcher dispatcher);

 public abstract void getAabb(btBroadphaseProxy proxy, final btVector3 aabbMin,
  final btVector3 aabbMax);

 public abstract void rayTest(final btVector3 rayFrom, final btVector3 rayTo,
  btBroadphaseRayCallback rayCallback, final btVector3 aabbMin, final btVector3 aabbMax);

 public void rayTest(final btVector3 rayFrom, final btVector3 rayTo,
  btBroadphaseRayCallback rayCallback) {
  rayTest(rayFrom, rayTo, rayCallback, new btVector3(), new btVector3());
 }

 public void rayTest(final btVector3 rayFrom, final btVector3 rayTo,
  btBroadphaseRayCallback rayCallback, final btVector3 aabbMin) {
  rayTest(rayFrom, rayTo, rayCallback, aabbMin, new btVector3());
 }

 public abstract void aabbTest(final btVector3 aabbMin, final btVector3 aabbMax,
  btBroadphaseAabbCallback callback);

 ///calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
 public abstract void calculateOverlappingPairs(btDispatcher dispatcher);

 public abstract btOverlappingPairCache getOverlappingPairCache();

 ///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
 ///will add some transform later
 public abstract void getBroadphaseAabb(final btVector3 aabbMin, final btVector3 aabbMax);

 ///reset broadphase internal structures, to ensure determinism/reproducability
 public void resetPool(btDispatcher dispatcher) {
  //(void) dispatcher; 
 }

 public abstract void printStats();
}
