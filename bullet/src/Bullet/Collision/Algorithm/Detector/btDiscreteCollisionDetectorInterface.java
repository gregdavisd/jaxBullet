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


package Bullet.Collision.Algorithm.Detector;

import Bullet.Collision.btIDebugDraw;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 * This interface is made to be used by an iterative approach to do TimeOfImpact calculations This
 * interface allows to query for closest points and penetration depth between two (convex) objects
 * the closest point is on the second object (B), and the normal points from the surface on B
 * towards A. distance is between closest points on B and closest point on A. So you can calculate
 * closest point on A by taking closestPointInA = closestPointInB + m_distance * m_normalOnSurfaceB
 *
 * @author Gregery Barton
 */
public abstract class btDiscreteCollisionDetectorInterface implements Serializable  {

 //
 // give either closest points (distance > 0) or penetration (distance)
 // the normal always points from B towards A
 //
 public final void getClosestPoints(ClosestPointInput input, Result output, btIDebugDraw debugDraw) {
  getClosestPoints(input, output, debugDraw, false);
 }

 public abstract void getClosestPoints(ClosestPointInput input, Result output, btIDebugDraw debugDraw,
  boolean swapResults);

 public static abstract class Result {

  ///setShapeIdentifiersA/B provides experimental support for per-triangle material / custom material combiner
 public  abstract void setShapeIdentifiersA(int partId0, int index0);

 public  abstract void setShapeIdentifiersB(int partId1, int index1);

public   abstract void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorld,
   float depth);
 }

 public static class ClosestPointInput implements Serializable{

  public final btTransform m_transformA = new btTransform();
  public final btTransform m_transformB = new btTransform();
  public float m_maximumDistanceSquared;

  public ClosestPointInput() {
   m_maximumDistanceSquared = BT_LARGE_FLOAT;
  }
 }
};
