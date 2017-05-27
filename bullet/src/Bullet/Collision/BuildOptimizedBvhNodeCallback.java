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

import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Gregery Barton
 */
class BuildOptimizedBvhNodeCallback implements btTriangleCallback,  Serializable  {

 private static final long serialVersionUID = 1L;

 final ArrayList<btOptimizedBvhNode> m_triangleNodes;

 BuildOptimizedBvhNodeCallback(ArrayList<btOptimizedBvhNode> triangleNodes) {
  m_triangleNodes = triangleNodes;
 }

 BuildOptimizedBvhNodeCallback set(BuildOptimizedBvhNodeCallback other) {
  m_triangleNodes.ensureCapacity(other.m_triangleNodes.size());
  m_triangleNodes.clear();
  for (btOptimizedBvhNode node : other.m_triangleNodes) {
   try {
    m_triangleNodes.add((btOptimizedBvhNode) node.clone());
   } catch (CloneNotSupportedException ex) {
    Logger.getLogger(BuildOptimizedBvhNodeCallback.class.getName()).log(Level.SEVERE, null, ex);
    assert(false);
   }
  }
  return this;
 }

 @Override
 public boolean processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  btOptimizedBvhNode node = new btOptimizedBvhNode();
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  aabbMin.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  aabbMax.set((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
  aabbMin.setMin(triangle[0]);
  aabbMax.setMax(triangle[0]);
  aabbMin.setMin(triangle[1]);
  aabbMax.setMax(triangle[1]);
  aabbMin.setMin(triangle[2]);
  aabbMax.setMax(triangle[2]);
  //with quantization?
  node.m_aabbMinOrg.set(aabbMin);
  node.m_aabbMaxOrg.set(aabbMax);
  node.m_escapeIndex = -1;
  //for child nodes
  node.m_subPart = partId;
  node.m_triangleIndex = triangleIndex;
  m_triangleNodes.add(node);
  return true;
 }
};
