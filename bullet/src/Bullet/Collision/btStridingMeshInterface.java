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

package Bullet.Collision;

import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
abstract public class btStridingMeshInterface  implements Serializable {

 final btVector3 m_scaling = new btVector3();
 final btInternalTriangleIndexCallback vertex_scaling;
 btInternalTriangleIndexCallback callback;

 btStridingMeshInterface() {
  m_scaling.set(1, 1, 1);
  vertex_scaling = new btInternalTriangleIndexCallback() {
   @Override
   public boolean internalProcessTriangleIndex(btVector3[] triangle, int partId, int triangleIndex) {
    for (int i = 0; i < 3; i++) {
     triangle[i].mul(getScaling());
    }
    callback.internalProcessTriangleIndex(triangle, partId, triangleIndex);
    return true;
   }
  };
 }

  public void InternalProcessAllTriangles(btInternalTriangleIndexCallback callback, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  this.callback = callback;
  int graphicssubparts = getNumSubParts();
  ///if the number of parts is big, the performance might drop due to the innerloop switch on indextype
  for (int part = 0; part < graphicssubparts; part++) {
   StridingMeshLock mesh = getLockedReadOnlyVertexIndexBase(part);
   mesh.process_all_triangles(vertex_scaling, part);
   //unLockReadOnlyVertexBase(part);
  }
 }

 void InternalProcessSubPart(btInternalTriangleIndexCallback callback, StridingMeshLock mesh,
  final btVector3 aabbMin, final btVector3 aabbMax, int part, int baseTriangleIndex) {
  this.callback = callback;
  mesh.process_all_triangles(vertex_scaling, part);
  //unLockReadOnlyVertexBase(part);
 }

 ///brute force method to calculate aabb
 void calculateAabbBruteForce(final btVector3 aabbMin, final btVector3 aabbMax) {
  //first calculate the total aabb for all triangles
  aabbMin.set((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
  aabbMax.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  btInternalTriangleIndexCallback aabbCallback = new btInternalTriangleIndexCallbackImpl(aabbMin,
   aabbMax);
  InternalProcessAllTriangles(aabbCallback, aabbMin, aabbMax);
 }

 /// get read and write access to a subpart of a triangle mesh
 /// this subpart has a continuous array of vertices and indices
 /// in this way the mesh can be handled as chunks of memory with striding
 /// very similar to OpenGL vertexarray support
 /// make a call to unLockVertexBase when the read and write access is finished	
 abstract StridingMeshLock getLockedReadOnlyVertexIndexBase(int subpart);

 StridingMeshLock getLockedReadOnlyVertexIndexBase() {
  return getLockedReadOnlyVertexIndexBase(0);
 }

 /// unLockVertexBase finishes the access to a subpart of the triangle mesh
 /// make a call to unLockVertexBase when the read and write access (using getLockedVertexIndexBase) is finished
 abstract void unLockVertexBase(int subpart);

 final void unLockReadOnlyVertexBase(int subpart) {
  // put  synchronization somewhere else if you must
  throw new AssertionError();
 }

 /// getNumSubParts returns the number of seperate subparts
 /// each subpart has a continuous array of vertices and indices
 abstract int getNumSubParts();

 abstract void preallocateVertices(int numverts);

 abstract void preallocateIndices(int numindices);

  public boolean hasPremadeAabb() {
  return false;
 }

 void setPremadeAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
 }

  public void getPremadeAabb(final btVector3 aabbMin, final btVector3 aabbMax) {
 }

  public btVector3 getScaling() {
  return new btVector3(m_scaling);
 }

  public void setScaling(final btVector3 scaling) {
  m_scaling.set(scaling);
 }

 private static class btInternalTriangleIndexCallbackImpl implements btInternalTriangleIndexCallback {

  private final btVector3 aabbMin;
  private final btVector3 aabbMax;

  public btInternalTriangleIndexCallbackImpl(final btVector3 aabbMin, final btVector3 aabbMax) {
   this.aabbMin = aabbMin;
   this.aabbMax = aabbMax;
  }

  @Override
  public boolean internalProcessTriangleIndex(btVector3[] triangle, int partId, int triangleIndex) {
   aabbMin.setMin(triangle[0]);
   aabbMax.setMax(triangle[0]);
   aabbMin.setMin(triangle[1]);
   aabbMax.setMax(triangle[1]);
   aabbMin.setMin(triangle[2]);
   aabbMax.setMax(triangle[2]);
   return true;
  }
 }
};
