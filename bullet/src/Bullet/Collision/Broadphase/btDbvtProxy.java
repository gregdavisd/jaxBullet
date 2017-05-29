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
///btDbvtBroadphase implementation by Nathanael Presson
package Bullet.Collision.Broadphase;

import static Bullet.Collision.Broadphase.btDbvt.Intersect;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
class btDbvtProxy extends btBroadphaseProxy implements Serializable {

 /* Fields		*/
 //btDbvtAabbMm	aabb;
 btDbvtNode leaf;
 int stage;

 /* ctor			*/
 btDbvtProxy(final btVector3 aabbMin, final btVector3 aabbMax, Object userPtr,
  int collisionFilterGroup,
  int collisionFilterMask) {
  super(aabbMin, aabbMax, userPtr, collisionFilterGroup, collisionFilterMask);
 }

 @Override
 public boolean intersect(btBroadphaseProxy other) {
  return Intersect(leaf.volume(), ((btDbvtProxy) other).leaf.volume());
 }
}
