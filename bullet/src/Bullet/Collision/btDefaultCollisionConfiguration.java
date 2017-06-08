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

import Bullet.Collision.Algorithm.btBoxBoxCollisionAlgorithm;
import Bullet.Collision.Algorithm.btCollisionAlgorithmCreateFunc;
import Bullet.Collision.Algorithm.btCompoundCollisionAlgorithm;
import Bullet.Collision.Algorithm.btCompoundCompoundCollisionAlgorithm;
import Bullet.Collision.Algorithm.btConvexConcaveCollisionAlgorithm;
import Bullet.Collision.Algorithm.btConvexConvexAlgorithm;
import Bullet.Collision.Algorithm.btConvexPlaneCollisionAlgorithm;
import Bullet.Collision.Algorithm.btEmptyAlgorithm;
import Bullet.Collision.Algorithm.btSphereSphereCollisionAlgorithm;
import Bullet.Collision.Algorithm.btSphereTriangleCollisionAlgorithm;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.BOX_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.SPHERE_SHAPE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.STATIC_PLANE_PROXYTYPE;
import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.TRIANGLE_SHAPE_PROXYTYPE;
import Bullet.Collision.Broadphase.btBroadphaseProxy;
import java.io.Serializable;

/**
 * btCollisionConfiguration allows to configure Bullet collision detection stack allocator, pool
 * memory allocators
 *
 * @todo: describe the meaning
 *
 * @author Gregery Barton
 */
public class btDefaultCollisionConfiguration implements btCollisionConfiguration, Serializable {

 //default penetration depth solver
 final public btConvexPenetrationDepthSolver m_pdSolver;
 final btCollisionAlgorithmCreateFunc m_convexConvexCreateFunc;
 final btCollisionAlgorithmCreateFunc m_convexConcaveCreateFunc;
 final btCollisionAlgorithmCreateFunc m_swappedConvexConcaveCreateFunc;
 final btCollisionAlgorithmCreateFunc m_compoundCreateFunc;
 final btCollisionAlgorithmCreateFunc m_compoundCompoundCreateFunc;
 final btCollisionAlgorithmCreateFunc m_swappedCompoundCreateFunc;
 final btCollisionAlgorithmCreateFunc m_emptyCreateFunc;
 final btCollisionAlgorithmCreateFunc m_sphereSphereCF;
//final  btCollisionAlgorithmCreateFunc m_sphereBoxCF;
//final  btCollisionAlgorithmCreateFunc m_boxSphereCF;
 final btCollisionAlgorithmCreateFunc m_boxBoxCF;
 final btCollisionAlgorithmCreateFunc m_sphereTriangleCF;
 final btCollisionAlgorithmCreateFunc m_triangleSphereCF;
 final btCollisionAlgorithmCreateFunc m_planeConvexCF;
 final btCollisionAlgorithmCreateFunc m_convexPlaneCF;

 public btDefaultCollisionConfiguration() {
  this(new btDefaultCollisionConstructionInfo());
 }

 public btDefaultCollisionConfiguration(btDefaultCollisionConstructionInfo constructionInfo) {
  if (constructionInfo.m_useEpaPenetrationAlgorithm) {
   m_pdSolver = new btGjkEpaPenetrationDepthSolver();
  } else {
   m_pdSolver = new btMinkowskiPenetrationDepthSolver();
  }
  //default CreationFunctions, filling the m_doubleDispatch table
  m_convexConvexCreateFunc = new btConvexConvexAlgorithm.CreateFunc(m_pdSolver);
  m_convexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm.CreateFunc();
  m_swappedConvexConcaveCreateFunc = new btConvexConcaveCollisionAlgorithm.SwappedCreateFunc();
  m_compoundCreateFunc = new btCompoundCollisionAlgorithm.CreateFunc();
  m_compoundCompoundCreateFunc = new btCompoundCompoundCollisionAlgorithm.CreateFunc();
  m_swappedCompoundCreateFunc = new btCompoundCollisionAlgorithm.SwappedCreateFunc();
  m_emptyCreateFunc = new btEmptyAlgorithm.CreateFunc();
  m_sphereSphereCF = new btSphereSphereCollisionAlgorithm.CreateFunc();
  m_sphereTriangleCF = new btSphereTriangleCollisionAlgorithm.CreateFunc();
  m_triangleSphereCF = new btSphereTriangleCollisionAlgorithm.CreateFunc();
  m_triangleSphereCF.m_swapped = true;
  m_boxBoxCF = new btBoxBoxCollisionAlgorithm.CreateFunc();
//  m_boxBoxCF = new btConvexConvexAlgorithm.CreateFunc(m_pdSolver);
//m_boxBoxCF=new btEmptyAlgorithm.CreateFunc();
  //convex versus plane
  m_convexPlaneCF = new btConvexPlaneCollisionAlgorithm.CreateFunc();
  m_planeConvexCF = new btConvexPlaneCollisionAlgorithm.CreateFunc();
  m_planeConvexCF.m_swapped = true;
 }

 ///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
 ///By default, this feature is disabled for best performance.
 ///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
 ///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
 ///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
 ///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
 ///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
 public final void setConvexConvexMultipointIterations() {
  setConvexConvexMultipointIterations(3, 3);
 }

 public final void setConvexConvexMultipointIterations(int numPerturbationIterations) {
  setConvexConvexMultipointIterations(numPerturbationIterations, 3);
 }

 public void setConvexConvexMultipointIterations(int numPerturbationIterations,
  int minimumPointsPerturbationThreshold) {
 }

 public final void setPlaneConvexMultipointIterations() {
  setPlaneConvexMultipointIterations(3, 3);
 }

 public final void setPlaneConvexMultipointIterations(int numPerturbationIterations) {
  setPlaneConvexMultipointIterations(numPerturbationIterations, 3);
 }

 void setPlaneConvexMultipointIterations(int numPerturbationIterations,
  int minimumPointsPerturbationThreshold) {
 }

 @Override
 public btCollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc(int proxyType0,
  int proxyType1) {
  if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE)) {
   return m_sphereSphereCF;
  }
  if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == TRIANGLE_SHAPE_PROXYTYPE)) {
   return m_sphereTriangleCF;
  }
  if ((proxyType0 == TRIANGLE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE)) {
   return m_triangleSphereCF;
  }
  if ((proxyType0 == BOX_SHAPE_PROXYTYPE) && (proxyType1 == BOX_SHAPE_PROXYTYPE)) {
   return m_boxBoxCF;
  }
  if (btBroadphaseProxy.isConvex(proxyType0) && (proxyType1 == STATIC_PLANE_PROXYTYPE)) {
   return m_convexPlaneCF;
  }
  if (btBroadphaseProxy.isConvex(proxyType1) && (proxyType0 == STATIC_PLANE_PROXYTYPE)) {
   return m_planeConvexCF;
  }
  if (btBroadphaseProxy.isConvex(proxyType0) && btBroadphaseProxy.isConvex(proxyType1)) {
   return m_convexConvexCreateFunc;
  }
  if (btBroadphaseProxy.isConvex(proxyType0) && btBroadphaseProxy.isConcave(proxyType1)) {
   return m_convexConcaveCreateFunc;
  }
  if (btBroadphaseProxy.isConvex(proxyType1) && btBroadphaseProxy.isConcave(proxyType0)) {
   return m_swappedConvexConcaveCreateFunc;
  }
  if (btBroadphaseProxy.isCompound(proxyType0) && btBroadphaseProxy.isCompound(proxyType1)) {
   return m_compoundCompoundCreateFunc;
  }
  if (btBroadphaseProxy.isCompound(proxyType0)) {
   return m_compoundCreateFunc;
  } else if (btBroadphaseProxy.isCompound(proxyType1)) {
   return m_swappedCompoundCreateFunc;
  }
  //failed to find an algorithm
  return m_emptyCreateFunc;
 }

 @Override
 public btCollisionAlgorithmCreateFunc getClosestPointsAlgorithmCreateFunc(int proxyType0,
  int proxyType1) {
  if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE)) {
   return m_sphereSphereCF;
  }
  if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1 == TRIANGLE_SHAPE_PROXYTYPE)) {
   return m_sphereTriangleCF;
  }
  if ((proxyType0 == TRIANGLE_SHAPE_PROXYTYPE) && (proxyType1 == SPHERE_SHAPE_PROXYTYPE)) {
   return m_triangleSphereCF;
  }
  if (btBroadphaseProxy.isConvex(proxyType0) && (proxyType1 == STATIC_PLANE_PROXYTYPE)) {
   return m_convexPlaneCF;
  }
  if (btBroadphaseProxy.isConvex(proxyType1) && (proxyType0 == STATIC_PLANE_PROXYTYPE)) {
   return m_planeConvexCF;
  }
  if (btBroadphaseProxy.isConvex(proxyType0) && btBroadphaseProxy.isConvex(proxyType1)) {
   return m_convexConvexCreateFunc;
  }
  if (btBroadphaseProxy.isConvex(proxyType0) && btBroadphaseProxy.isConcave(proxyType1)) {
   return m_convexConcaveCreateFunc;
  }
  if (btBroadphaseProxy.isConvex(proxyType1) && btBroadphaseProxy.isConcave(proxyType0)) {
   return m_swappedConvexConcaveCreateFunc;
  }
  if (btBroadphaseProxy.isCompound(proxyType0) && btBroadphaseProxy.isCompound(proxyType1)) {
   return m_compoundCompoundCreateFunc;
  }
  if (btBroadphaseProxy.isCompound(proxyType0)) {
   return m_compoundCreateFunc;
  } else if (btBroadphaseProxy.isCompound(proxyType1)) {
   return m_swappedCompoundCreateFunc;
  }
  //failed to find an algorithm
  return m_emptyCreateFunc;
 }
}
