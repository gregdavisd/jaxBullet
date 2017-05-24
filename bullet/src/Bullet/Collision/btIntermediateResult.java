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

package Bullet.Collision;

import Bullet.Collision.Algorithm.Detector.btDiscreteCollisionDetectorInterface;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
class btIntermediateResult extends btDiscreteCollisionDetectorInterface.Result  implements Serializable {

 btIntermediateResult() {
  m_hasResult = false;
 }
 final btVector3 m_normalOnBInWorld = new btVector3();
 final btVector3 m_pointInWorld = new btVector3();
 float m_depth;
 boolean m_hasResult;

 @Override
public  void setShapeIdentifiersA(int partId0, int index0) {
 }

 @Override
 public void setShapeIdentifiersB(int partId1, int index1) {
 }

 @Override
public  void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorld, float depth) {
  m_normalOnBInWorld.set(normalOnBInWorld);
  m_pointInWorld.set(pointInWorld);
  m_depth = depth;
  m_hasResult = true;
 }
};
