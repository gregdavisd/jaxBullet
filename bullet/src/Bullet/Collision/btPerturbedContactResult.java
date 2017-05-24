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

import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btPerturbedContactResult extends btManifoldResult  implements Serializable {

 final btManifoldResult m_originalManifoldResult;
 final btTransform m_transformA = new btTransform();
 final btTransform m_transformB = new btTransform();
 final btTransform m_unPerturbedTransform = new btTransform();
 boolean m_perturbA;
 final btIDebugDraw m_debugDrawer;

 public btPerturbedContactResult(btManifoldResult originalResult, final btTransform transformA,
  final btTransform transformB, final btTransform unPerturbedTransform, boolean perturbA,
  btIDebugDraw debugDrawer) {
  m_originalManifoldResult = originalResult;
  m_transformA.set(transformA);
  m_transformB.set(transformB);
  m_unPerturbedTransform.set(unPerturbedTransform);
  m_perturbA = perturbA;
  m_debugDrawer = debugDrawer;
 }

 @Override
 public void addContactPoint(final btVector3 normalOnBInWorld, final btVector3 pointInWorld, float orgDepth) {
  final btVector3 endPt = new btVector3();
  final btVector3 startPt = new btVector3();
  float newDepth;
  if (m_perturbA) {
   final btVector3 endPtOrg = new btVector3(new btVector3(pointInWorld).add(new btVector3(
    normalOnBInWorld).scale(orgDepth)));
   endPt.set((new btTransform(m_unPerturbedTransform).mul(new btTransform(m_transformA).invert()))
    .transform(new btVector3(endPtOrg)));
   newDepth = (new btVector3(endPt).sub(pointInWorld)).dot(normalOnBInWorld);
   startPt.scaleAdd(newDepth, normalOnBInWorld, endPt);
  } else {
   endPt.scaleAdd(orgDepth, normalOnBInWorld, pointInWorld);
   startPt.set((new btTransform(m_unPerturbedTransform).mul(new btTransform(m_transformB).invert()))
    .transform(new btVector3(pointInWorld)));
   newDepth = (new btVector3(endPt).sub(startPt)).dot(normalOnBInWorld);
  }
  m_originalManifoldResult.addContactPoint(normalOnBInWorld, startPt, newDepth);
 }
}
