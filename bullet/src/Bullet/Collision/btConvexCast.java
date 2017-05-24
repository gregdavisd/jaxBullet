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

import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public abstract class btConvexCast  implements Serializable {

 /// cast a convex against another convex object
 abstract boolean calcTimeOfImpact(
  final btTransform fromA, final btTransform toA, final btTransform fromB, final btTransform toB,
  CastResult result);

 ///RayResult stores the closest result
 /// alternatively, add a callback method to decide about closest/all results
 public static class CastResult {

public   final btTransform m_hitTransformA = new btTransform();
public   final btTransform m_hitTransformB = new btTransform();
public   final btVector3 m_normal = new btVector3();
public   final btVector3 m_hitPoint = new btVector3();
public   float m_fraction; //input and output
public   btIDebugDraw m_debugDrawer;
public   float m_allowedPenetration;

  public CastResult() {
   m_fraction = (BT_LARGE_FLOAT);
  }
  //  boolean	addRayResult(  btVector3& normal,float	fraction) = 0;

public   void DebugDraw(float fraction) {
  }

public   void drawCoordSystem(final btTransform trans) {
  }

public   void reportFailure(int errNo, int numIterations) {
  }
 }
};
