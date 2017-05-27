/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

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

import Bullet.Collision.Shape.btStridingMeshLock;
import Bullet.Collision.Shape.btStridingMeshInterface;
import Bullet.Collision.Broadphase.btNodeOverlapCallback;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class MyNodeOverlapCallbackConvexcast implements btNodeOverlapCallback ,   Serializable {

 final btStridingMeshInterface m_meshInterface;
 final btTriangleCallback m_callback;
 final btTriangleCallback vert_scaling_callback;
 final btVector3 mesh_scaling = new btVector3();
 int part = -1;
 btStridingMeshLock mesh_lock;

 public MyNodeOverlapCallbackConvexcast(btTriangleCallback callback, btStridingMeshInterface meshInterface) {
  m_meshInterface = meshInterface;
  m_callback = callback;
  vert_scaling_callback = new btTriangleCallback() {
   private static final long serialVersionUID = 1L;
   @Override
   public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
    for (int i = 0; i < 3; i++) {
     triangle[i].mul(mesh_scaling);
    }
    m_callback.processTriangle(triangle, partId, triangleIndex);
    // only process one triangle
    return false;
   }
  };
 }

 /**
  *
  * @param nodeSubPart
  * @param nodeTriangleIndex
  */
 @Override
 public void processNode(int nodeSubPart, int nodeTriangleIndex) {
  btVector3[] m_triangle = new btVector3[3];
  init(m_triangle);

  /* TODO: creating a lock for every triangle is going to be slow
   */
  if (nodeSubPart != part) {
   mesh_lock = m_meshInterface.getLockedReadOnlyVertexIndexBase(nodeSubPart);
   part = nodeSubPart;
  }
  mesh_scaling.set(m_meshInterface.getScaling());

  /* Perform ray vs. triangle collision here */
  mesh_lock.process_all_triangles(vert_scaling_callback, nodeSubPart, nodeTriangleIndex);
  //m_meshInterface.unLockReadOnlyVertexBase(nodeSubPart);
 }
}
