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
package Bullet.Collision.Algorithm;

import Bullet.Collision.Broadphase.btDispatcherInfo;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btCollisionObjectWrapper;
import Bullet.Collision.btManifoldResult;
import Bullet.Collision.btPersistentManifold;
import java.io.Serializable;
import java.util.ArrayList;

/**
 * EmptyAlgorithm is a stub for unsupported collision pairs. The dispatcher can dispatch a
 * persistent btEmptyAlgorithm to avoid a search every frame.
 *
 * @author Gregery Barton
 */
public class btEmptyAlgorithm extends btCollisionAlgorithm implements Serializable {

 public btEmptyAlgorithm(btCollisionAlgorithmConstructionInfo ci) {
  super(ci);
 }

 @Override
 public void processCollision(btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
 }

 @Override
 public float calculateTimeOfImpact(btCollisionObject body0, btCollisionObject body1,
  btDispatcherInfo dispatchInfo, btManifoldResult resultOut) {
  return (1.f);
 }

 @Override
 public void getAllContactManifolds(ArrayList<btPersistentManifold> manifoldArray) {
 }

 @Override
 public void destroy() {
 }

 public static class CreateFunc extends btCollisionAlgorithmCreateFunc {

  @Override
  public btCollisionAlgorithm CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo ci,
   btCollisionObjectWrapper body0Wrap, btCollisionObjectWrapper body1Wrap) {
   return new btEmptyAlgorithm(ci);
  }
 };
}
