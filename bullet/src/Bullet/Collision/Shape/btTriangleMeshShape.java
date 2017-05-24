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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_MESH_SHAPE_PROXYTYPE;
import Bullet.Collision.FilteredCallback;
import Bullet.Collision.SupportVertexCallback;
import Bullet.Collision.btStridingMeshInterface;
import Bullet.Collision.btTriangleCallback;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btTriangleMeshShape extends btConcaveShape  implements Serializable {

 final btVector3 m_localAabbMin;
 final btVector3 m_localAabbMax;
 final btStridingMeshInterface m_meshInterface;

 ///btTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
 ///Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
 btTriangleMeshShape(btStridingMeshInterface meshInterface) {
  super();
  m_meshInterface = meshInterface;
  m_shapeType = TRIANGLE_MESH_SHAPE_PROXYTYPE;
  m_localAabbMin = new btVector3();
  m_localAabbMax = new btVector3();
  if (meshInterface.hasPremadeAabb()) {
   meshInterface.getPremadeAabb(m_localAabbMin, m_localAabbMax);
  } else {
   recalcLocalAabb();
  }
 }

 btVector3 localGetSupportingVertex(final btVector3 vec) {
  final btVector3 supportVertex;
  final btTransform ident = new btTransform();
  ident.setIdentity();
  SupportVertexCallback supportCallback = new SupportVertexCallback(vec, ident);
  final btVector3 aabbMax = new btVector3((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  processAllTriangles(supportCallback, new btVector3(aabbMax).negate(), aabbMax);
  supportVertex = supportCallback.getSupportVertexLocal();
  return supportVertex;
 }

 btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec) {
  assert(false);
  return localGetSupportingVertex(vec);
 }

 final void recalcLocalAabb() {
  for (int i = 0; i < 3; i++) {
   final btVector3 vec = new btVector3();
   vec.setElement(i, 1.f);
   final btVector3 tmp = localGetSupportingVertex(vec);
   m_localAabbMax.setElement(i, tmp.getElement(i) + m_collisionMargin);
   vec.setElement(i, -1.f);
   tmp.set(localGetSupportingVertex(vec));
   m_localAabbMin.setElement(i, tmp.getElement(i) - m_collisionMargin);
  }
 }

 @Override
 public void getAabb(final btTransform trans, final btVector3 aabbMin, final btVector3 aabbMax) {
  final btVector3 localHalfExtents = new btVector3(m_localAabbMax).sub(m_localAabbMin).scale(0.5f);
  localHalfExtents.add(new btVector3(getMargin(), getMargin(), getMargin()));
  final btVector3 localCenter = new btVector3(m_localAabbMax).add(m_localAabbMin).scale(0.5f);
  final btMatrix3x3 abs_b = trans.getBasis().abs();
  final btVector3 center = trans.transform(new btVector3(localCenter));
  final btVector3 extent = localHalfExtents.dot3(abs_b.getRow(0), abs_b.getRow(1), abs_b.getRow(2));
  aabbMin.set(center).sub(extent);
  aabbMax.set(center).add(extent);
 }

 @Override
 public void processAllTriangles(btTriangleCallback callback, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  FilteredCallback filterCallback = new FilteredCallback(callback, aabbMin, aabbMax);
  m_meshInterface.InternalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //moving concave objects not supported
  assert(false);
  inertia.set(0, 0, 0);
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  m_meshInterface.setScaling(scaling);
  recalcLocalAabb();
 }

 @Override
 public btVector3 getLocalScaling() {
  return m_meshInterface.getScaling();
 }

 btStridingMeshInterface getMeshInterface() {
  return m_meshInterface;
 }

 btVector3 getLocalAabbMin() {
  return new btVector3(m_localAabbMin);
 }

 btVector3 getLocalAabbMax() {
  return new btVector3(m_localAabbMax);
 }

 //debugging
 @Override
 public String getName() {
  return "TRIANGLEMESH";
 }
};
