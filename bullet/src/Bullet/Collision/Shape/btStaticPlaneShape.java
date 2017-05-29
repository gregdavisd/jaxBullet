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
package Bullet.Collision.Shape;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE;
import Bullet.Collision.btTriangleCallback;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btStaticPlaneShape extends btConcaveShape implements Serializable {

 final btVector3 m_localAabbMin = new btVector3();
 final btVector3 m_localAabbMax = new btVector3();
 final btVector3 m_planeNormal = new btVector3();
 float m_planeConstant;
 final btVector3 m_localScaling = new btVector3();

 public btStaticPlaneShape(final btVector3 planeNormal, float planeConstant) {
  super();
  m_planeNormal.set(planeNormal).normalize();
  m_planeConstant = planeConstant;
  m_localScaling.set(1.f, 1f, 1f);
  m_shapeType = STATIC_PLANE_PROXYTYPE;
 }

 @Override
 public void getAabb(final btTransform t, final btVector3 aabbMin, final btVector3 aabbMax) {
  aabbMin.set(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
  aabbMax.set(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
 }

 @Override
 public void processAllTriangles(btTriangleCallback callback, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  final btVector3 halfExtents = (new btVector3(aabbMax).sub(aabbMin)).scale(0.5f);
  float radius = halfExtents.length();
  final btVector3 center = new btVector3(aabbMax).add(aabbMin).scale(0.5f);
  //this is where the triangles are generated, given AABB and plane equation (normal/constant)
  final btVector3 tangentDir0 = new btVector3();
  final btVector3 tangentDir1 = new btVector3();
  //tangentDir0/tangentDir1 can be precalculated
  btPlaneSpace1(m_planeNormal, tangentDir0, tangentDir1);
  final btVector3 projectedCenter = new btVector3(center).sub(new btVector3(m_planeNormal).scale(
   m_planeNormal
   .dot(center) - m_planeConstant));
  btVector3[] triangle = new btVector3[3];
  init(triangle);
  triangle[0].set(projectedCenter).add(new btVector3(tangentDir0).scale(radius)).add(
   new btVector3(tangentDir1).scale(radius));
  triangle[1].set(projectedCenter).add(new btVector3(tangentDir0).scale(radius)).sub(
   new btVector3(tangentDir1).scale(radius));
  triangle[2].set(projectedCenter).sub(new btVector3(tangentDir0).scale(radius)).sub(
   new btVector3(tangentDir1).scale(radius));
  callback.processTriangle(triangle, 0, 0);
  triangle[0].set(projectedCenter).sub(new btVector3(tangentDir0).scale(radius)).sub(
   new btVector3(tangentDir1).scale(radius));
  triangle[1].set(projectedCenter).sub(new btVector3(tangentDir0).scale(radius)).add(
   new btVector3(tangentDir1).scale(radius));
  triangle[2].set(projectedCenter).add(new btVector3(tangentDir0).scale(radius)).add(
   new btVector3(tangentDir1).scale(radius));
  callback.processTriangle(triangle, 0, 1);
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //moving concave objects not supported
  inertia.set(0.f, 0f, 0f);
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  m_localScaling.set(scaling);
 }

 @Override
 public btVector3 getLocalScaling() {
  return new btVector3(m_localScaling);
 }

 public btVector3 getPlaneNormal() {
  return new btVector3(m_planeNormal);
 }

 public float getPlaneConstant() {
  return m_planeConstant;
 }

 //debugging
 @Override
 public String getName() {
  return "STATICPLANE";
 }
}
