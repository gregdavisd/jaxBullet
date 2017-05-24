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
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class SupportVertexCallback implements btTriangleCallback,   Serializable  {

  public final btVector3 m_supportVertexLocal;
  public final btTransform m_worldTrans;
  public float m_maxDot;
  public final btVector3 m_supportVecLocal;

  public SupportVertexCallback(final btVector3 supportVecWorld, final btTransform trans) {
  m_supportVertexLocal = new btVector3();
  m_worldTrans = new btTransform(trans);
  m_maxDot = (-BT_LARGE_FLOAT);
  m_supportVecLocal = m_worldTrans.transposeTransform3x3(new btVector3(supportVecWorld));
 }

 /**
  *
  * @param triangle
  * @param partId
  * @param triangleIndex
  */
 @Override
 public void processTriangle(btVector3[] triangle, int partId, int triangleIndex) {
  for (int i = 0; i < 3; i++) {
   float dot = m_supportVecLocal.dot(triangle[i]);
   if (dot > m_maxDot) {
    m_maxDot = dot;
    m_supportVertexLocal.set(triangle[i]);
   }
  }
 }

  public btVector3 getSupportVertexWorldSpace() {
  return m_worldTrans.transform(new btVector3(m_supportVertexLocal));
 }

  public btVector3 getSupportVertexLocal() {
  return new btVector3(m_supportVertexLocal);
 }
};
