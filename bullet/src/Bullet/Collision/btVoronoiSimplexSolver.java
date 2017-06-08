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
import static Bullet.LinearMath.btScalar.FLT_MAX;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import static Bullet.common.IFDEF.CATCH_DEGENERATE_TETRAHEDRON;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class btVoronoiSimplexSolver implements btSimplexSolverInterface, Serializable {

 static final int VORONOI_SIMPLEX_MAX_VERTS = 5;
 static final float VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD = 0.0001f;
 static final int VERTA = 0;
 static final int VERTB = 1;
 static final int VERTC = 2;
 static final int VERTD = 3;
 int m_numVertices;
 final btVector3[] m_simplexVectorW = new btVector3[VORONOI_SIMPLEX_MAX_VERTS];
 final btVector3[] m_simplexPointsP = new btVector3[VORONOI_SIMPLEX_MAX_VERTS];
 final btVector3[] m_simplexPointsQ = new btVector3[VORONOI_SIMPLEX_MAX_VERTS];
 final btVector3 m_cachedP1 = new btVector3();
 final btVector3 m_cachedP2 = new btVector3();
 final btVector3 m_cachedV = new btVector3();
 final btVector3 m_lastW = new btVector3();
 float m_equalVertexThreshold;
 boolean m_cachedValidClosest;
 final btSubSimplexClosestResult m_cachedBC;
 boolean m_needsUpdate;

 public btVoronoiSimplexSolver() {
  this.m_cachedBC = new btSubSimplexClosestResult();
  m_equalVertexThreshold = VORONOI_DEFAULT_EQUAL_VERTEX_THRESHOLD;
  init(m_simplexVectorW);
  init(m_simplexPointsP);
  init(m_simplexPointsQ);
 }

 void removeVertex(int index) {
  assert (m_numVertices > 0);
  m_numVertices--;
  m_simplexVectorW[index].set(m_simplexVectorW[m_numVertices]);
  m_simplexPointsP[index].set(m_simplexPointsP[m_numVertices]);
  m_simplexPointsQ[index].set(m_simplexPointsQ[m_numVertices]);
//  if (DEBUG_BLOCKS) {
//   m_simplexVectorW[m_numVertices].set(new btVector3(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT,
//    -BT_LARGE_FLOAT));
//   m_simplexPointsP[m_numVertices].set(new btVector3(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT,
//    -BT_LARGE_FLOAT));
//   m_simplexPointsQ[m_numVertices].set(new btVector3(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT,
//    -BT_LARGE_FLOAT));
//  }
 }

 void reduceVertices(btUsageBitfield usedVerts) {
  if ((numVertices() >= 4) && (!usedVerts.usedVertexD)) {
   removeVertex(3);
  }
  if ((numVertices() >= 3) && (!usedVerts.usedVertexC)) {
   removeVertex(2);
  }
  if ((numVertices() >= 2) && (!usedVerts.usedVertexB)) {
   removeVertex(1);
  }
  if ((numVertices() >= 1) && (!usedVerts.usedVertexA)) {
   removeVertex(0);
  }
 }

 boolean updateClosestVectorAndPoints() {
  if (m_needsUpdate) {
   m_cachedBC.reset();
   m_needsUpdate = false;
   switch (numVertices()) {
    case 0:
     m_cachedValidClosest = false;
     break;
    case 1: {
     m_cachedP1.set(m_simplexPointsP[0]);
     m_cachedP2.set(m_simplexPointsQ[0]);
     m_cachedV.set(m_cachedP1).sub(m_cachedP2); //== m_simplexVectorW[0]
     m_cachedBC.reset();
     m_cachedBC.setBarycentricCoordinates(1.f, (0.f), (0.f), (0.f));
     m_cachedValidClosest = m_cachedBC.isValid();
     break;
    }
    case 2: {
     //closest point origin from line segment
     final btVector3 from = m_simplexVectorW[0];
     final btVector3 to = m_simplexVectorW[1];
     final btVector3 nearest = new btVector3();
     final btVector3 p = new btVector3();
     final btVector3 diff = new btVector3(p).sub(from);
     final btVector3 v = new btVector3(to).sub(from);
     float t = v.dot(diff);
     if (t > 0) {
      float dotVV = v.dot(v);
      if (t < dotVV) {
       t /= dotVV;
       diff.sub(new btVector3(v).scale(t));
       m_cachedBC.m_usedVertices.usedVertexA = true;
       m_cachedBC.m_usedVertices.usedVertexB = true;
      } else {
       t = 1;
       diff.sub(v);
       //reduce to 1 point
       m_cachedBC.m_usedVertices.usedVertexB = true;
      }
     } else {
      t = 0;
      //reduce to 1 point
      m_cachedBC.m_usedVertices.usedVertexA = true;
     }
     m_cachedBC.setBarycentricCoordinates(1 - t, t);
     nearest.scaleAdd(t, v, from);
     m_cachedP1.set(m_simplexPointsP[1]).sub(m_simplexPointsP[0]).scale(t).add(m_simplexPointsP[0]);
     m_cachedP2.set(m_simplexPointsQ[1]).sub(m_simplexPointsQ[0]).scale(t).add(m_simplexPointsQ[0]);
     m_cachedV.set(m_cachedP1).sub(m_cachedP2);
     reduceVertices(m_cachedBC.m_usedVertices);
     m_cachedValidClosest = m_cachedBC.isValid();
     break;
    }
    case 3: {
     //closest point origin from triangle 
     final btVector3 p = new btVector3();
     final btVector3 a = m_simplexVectorW[0];
     final btVector3 b = m_simplexVectorW[1];
     final btVector3 c = m_simplexVectorW[2];
     closestPtPointTriangle(p, a, b, c, m_cachedBC);
     m_cachedP1
      .set(m_simplexPointsP[0])
      .scale(m_cachedBC.m_barycentricCoords[0])
      .add(new btVector3(m_simplexPointsP[1])
       .scale(m_cachedBC.m_barycentricCoords[1]))
      .add(new btVector3(m_simplexPointsP[2])
       .scale(m_cachedBC.m_barycentricCoords[2]));
     m_cachedP2
      .set(m_simplexPointsQ[0])
      .scale(m_cachedBC.m_barycentricCoords[0])
      .add(new btVector3(m_simplexPointsQ[1])
       .scale(m_cachedBC.m_barycentricCoords[1]))
      .add(new btVector3(m_simplexPointsQ[2])
       .scale(m_cachedBC.m_barycentricCoords[2]));
     m_cachedV.set(m_cachedP1).sub(m_cachedP2);
     reduceVertices(m_cachedBC.m_usedVertices);
     m_cachedValidClosest = m_cachedBC.isValid();
     break;
    }
    case 4: {
     final btVector3 p = new btVector3();
     final btVector3 a = m_simplexVectorW[0];
     final btVector3 b = m_simplexVectorW[1];
     final btVector3 c = m_simplexVectorW[2];
     final btVector3 d = m_simplexVectorW[3];
     boolean hasSeperation = closestPtPointTetrahedron(p, a, b, c, d, m_cachedBC);
     if (hasSeperation) {
      m_cachedP1
       .set(m_simplexPointsP[0])
       .scale(m_cachedBC.m_barycentricCoords[0])
       .add(new btVector3(m_simplexPointsP[1])
        .scale(m_cachedBC.m_barycentricCoords[1]))
       .add(new btVector3(m_simplexPointsP[2])
        .scale(m_cachedBC.m_barycentricCoords[2]))
       .add(new btVector3(m_simplexPointsP[3])
        .scale(m_cachedBC.m_barycentricCoords[3]));
      m_cachedP2
       .set(m_simplexPointsQ[0])
       .scale(m_cachedBC.m_barycentricCoords[0])
       .add(new btVector3(m_simplexPointsQ[1])
        .scale(m_cachedBC.m_barycentricCoords[1]))
       .add(new btVector3(m_simplexPointsQ[2])
        .scale(m_cachedBC.m_barycentricCoords[2]))
       .add(new btVector3(m_simplexPointsQ[3])
        .scale(m_cachedBC.m_barycentricCoords[3]));
      m_cachedV
       .set(m_cachedP1)
       .sub(m_cachedP2);
      reduceVertices(m_cachedBC.m_usedVertices);
     } else {
//					printf("sub distance got penetration\n");
      if (m_cachedBC.m_degenerate) {
       m_cachedValidClosest = false;
      } else {
       m_cachedValidClosest = true;
       //degenerate case == false, penetration = true + zero
       m_cachedV.set(0, 0, 0);
      }
      break;
     }
     m_cachedValidClosest = m_cachedBC.isValid();
     //closest point origin from tetrahedron
     break;
    }
    default: {
     m_cachedValidClosest = false;
    }
   }
  }
  return m_cachedValidClosest;
 }

 boolean closestPtPointTetrahedron(final btVector3 p, final btVector3 a, final btVector3 b,
  final btVector3 c, final btVector3 d,
  btSubSimplexClosestResult finalResult) {
  btSubSimplexClosestResult tempResult = new btSubSimplexClosestResult();
  // Start out assuming point inside all halfspaces, so closest to itself
  finalResult.m_closestPointOnSimplex.set(p);
  finalResult.m_usedVertices.reset();
  finalResult.m_usedVertices.usedVertexA = true;
  finalResult.m_usedVertices.usedVertexB = true;
  finalResult.m_usedVertices.usedVertexC = true;
  finalResult.m_usedVertices.usedVertexD = true;
  int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
  int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
  int pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
  int pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);
  if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0) {
   finalResult.m_degenerate = true;
   return false;
  }
  if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0) {
   return false;
  }
  float bestSqDist = FLT_MAX;
  // If point outside face abc then compute closest point on abc
  if (pointOutsideABC != 0) {
   closestPtPointTriangle(p, a, b, c, tempResult);
   final btVector3 q = tempResult.m_closestPointOnSimplex;
   float sqDist = p.distanceSquared(q);
   // Update best closest point if (squared) distance is less than current best
   if (sqDist < bestSqDist) {
    bestSqDist = sqDist;
    finalResult.m_closestPointOnSimplex.set(q);
    //convert result bitmask!
    finalResult.m_usedVertices.reset();
    finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
    finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexB;
    finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
    finalResult.setBarycentricCoordinates(
     tempResult.m_barycentricCoords[VERTA],
     tempResult.m_barycentricCoords[VERTB],
     tempResult.m_barycentricCoords[VERTC],
     0
    );
   }
  }
  // Repeat test for face acd
  if (pointOutsideACD != 0) {
   closestPtPointTriangle(p, a, c, d, tempResult);
   final btVector3 q = tempResult.m_closestPointOnSimplex;
   //convert result bitmask!
   float sqDist = p.distanceSquared(q);
   if (sqDist < bestSqDist) {
    bestSqDist = sqDist;
    finalResult.m_closestPointOnSimplex.set(q);
    finalResult.m_usedVertices.reset();
    finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
    finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexB;
    finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexC;
    finalResult.setBarycentricCoordinates(
     tempResult.m_barycentricCoords[VERTA],
     0,
     tempResult.m_barycentricCoords[VERTB],
     tempResult.m_barycentricCoords[VERTC]
    );
   }
  }
  // Repeat test for face adb
  if (pointOutsideADB != 0) {
   closestPtPointTriangle(p, a, d, b, tempResult);
   final btVector3 q = tempResult.m_closestPointOnSimplex;
   //convert result bitmask!
   float sqDist = p.distanceSquared(q);
   if (sqDist < bestSqDist) {
    bestSqDist = sqDist;
    finalResult.m_closestPointOnSimplex.set(q);
    finalResult.m_usedVertices.reset();
    finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
    finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexC;
    finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
    finalResult.setBarycentricCoordinates(
     tempResult.m_barycentricCoords[VERTA],
     tempResult.m_barycentricCoords[VERTC],
     0,
     tempResult.m_barycentricCoords[VERTB]
    );
   }
  }
  // Repeat test for face bdc
  if (pointOutsideBDC != 0) {
   closestPtPointTriangle(p, b, d, c, tempResult);
   final btVector3 q = tempResult.m_closestPointOnSimplex;
   //convert result bitmask!
   float sqDist = p.distanceSquared(q);
   if (sqDist < bestSqDist) {
    finalResult.m_closestPointOnSimplex.set(q);
    finalResult.m_usedVertices.reset();
    //
    finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexA;
    finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
    finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
    finalResult.setBarycentricCoordinates(
     0,
     tempResult.m_barycentricCoords[VERTA],
     tempResult.m_barycentricCoords[VERTC],
     tempResult.m_barycentricCoords[VERTB]
    );
   }
  }
  //help! we ended up full !
//  if (finalResult.m_usedVertices.usedVertexA && finalResult.m_usedVertices.usedVertexB &&
//    finalResult.m_usedVertices.usedVertexC && finalResult.m_usedVertices.usedVertexD) {
//   return true;
//  }
  return true;
 }

 int pointOutsideOfPlane(final btVector3 p, final btVector3 a, final btVector3 b, final btVector3 c,
  final btVector3 d) {
  final btVector3 normal = (new btVector3(b).sub(a)).cross(new btVector3(c).sub(a));
  float signp = (new btVector3(p).sub(a)).dot(normal); // [AP AB AC]
  float signd = (new btVector3(d).sub(a)).dot(normal); // [AD AB AC]
  if (CATCH_DEGENERATE_TETRAHEDRON) {
   if (signd * signd < ((1e-4) * (1e-4))) {
    //System.err.println("affine dependent/degenerate\n");
    return -1;
   }
  }
  // Points on opposite sides if expression signs are opposite
  return (signp * signd < (0.f)) ? 1 : 0;
 }

 boolean closestPtPointTriangle(final btVector3 p, final btVector3 a, final btVector3 b,
  final btVector3 c,
  btSubSimplexClosestResult result) {
  result.m_usedVertices.reset();
  // Check if P in vertex region outside A
  final btVector3 ab = new btVector3(b).sub(a);
  final btVector3 ac = new btVector3(c).sub(a);
  final btVector3 ap = new btVector3(p).sub(a);
  float d1 = ab.dot(ap);
  float d2 = ac.dot(ap);
  if (d1 <= (0.0f) && d2 <= (0.0f)) {
   result.m_closestPointOnSimplex.set(a);
   result.m_usedVertices.usedVertexA = true;
   result.setBarycentricCoordinates(1, 0, 0);
   return true;// a; // barycentric coordinates (1,0,0)
  }
  // Check if P in vertex region outside B
  final btVector3 bp = new btVector3(p).sub(b);
  float d3 = ab.dot(bp);
  float d4 = ac.dot(bp);
  if ((d3 >= (0.0f)) && (d4 <= d3)) {
   result.m_closestPointOnSimplex.set(b);
   result.m_usedVertices.usedVertexB = true;
   result.setBarycentricCoordinates(0, 1, 0);
   return true; // b; // barycentric coordinates (0,1,0)
  }
  // Check if P in edge region of AB, if so return projection of P onto AB
  float vc = d1 * d4 - d3 * d2;
  if (vc <= (0.0f) && d1 >= (0.0f) && d3 <= (0.0f)) {
   float v = d1 / (d1 - d3);
   result.m_closestPointOnSimplex.scaleAdd(v, ab, a);
   result.m_usedVertices.usedVertexA = true;
   result.m_usedVertices.usedVertexB = true;
   result.setBarycentricCoordinates(1 - v, v, 0);
   return true;
   //return a + v * ab; // barycentric coordinates (1-v,v,0)
  }
  // Check if P in vertex region outside C
  final btVector3 cp = new btVector3(p).sub(c);
  float d5 = ab.dot(cp);
  float d6 = ac.dot(cp);
  if (d6 >= (0.0f) && d5 <= d6) {
   result.m_closestPointOnSimplex.set(c);
   result.m_usedVertices.usedVertexC = true;
   result.setBarycentricCoordinates(0, 0, 1);
   return true;//c; // barycentric coordinates (0,0,1)
  }
  // Check if P in edge region of AC, if so return projection of P onto AC
  float vb = d5 * d2 - d1 * d6;
  if (vb <= (0.0f) && d2 >= (0.0f) && d6 <= (0.0f)) {
   float w = d2 / (d2 - d6);
   result.m_closestPointOnSimplex.scaleAdd(w, ac, a);
   result.m_usedVertices.usedVertexA = true;
   result.m_usedVertices.usedVertexC = true;
   result.setBarycentricCoordinates(1 - w, 0, w);
   return true;
   //return a + w * ac; // barycentric coordinates (1-w,0,w)
  }
  // Check if P in edge region of BC, if so return projection of P onto BC
  float va = d3 * d6 - d5 * d4;
  if (va <= (0.0f) && (d4 - d3) >= (0.0f) && (d5 - d6) >= (0.0f)) {
   float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
   result.m_closestPointOnSimplex.set(c).sub(b).scale(w).add(b);
   result.m_usedVertices.usedVertexB = true;
   result.m_usedVertices.usedVertexC = true;
   result.setBarycentricCoordinates(0, 1 - w, w);
   return true;
   // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
  }
  // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
  float denom = (1.0f) / (va + vb + vc);
  float v = vb * denom;
  float w = vc * denom;
  result.m_closestPointOnSimplex
   .set(a)
   .add(new btVector3(ab)
    .scale(v))
   .add(new btVector3(ac)
    .scale(w));
  result.m_usedVertices.usedVertexA = true;
  result.m_usedVertices.usedVertexB = true;
  result.m_usedVertices.usedVertexC = true;
  result.setBarycentricCoordinates(1.0f - v - w, v, w);
  return true;
//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = float(1.0) - v - w
 }

 /**
  *
  */
 @Override
 public void reset() {
  m_cachedValidClosest = false;
  m_numVertices = 0;
  m_needsUpdate = true;
  m_lastW.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_cachedBC.reset();
 }

 /**
  *
  * @param w
  * @param p
  * @param q
  */
 @Override
 public void addVertex(final btVector3 w, final btVector3 p, final btVector3 q) {
  m_lastW.set(w);
  m_needsUpdate = true;
  m_simplexVectorW[m_numVertices].set(w);
  m_simplexPointsP[m_numVertices].set(p);
  m_simplexPointsQ[m_numVertices].set(q);
  m_numVertices++;
 }

 void setEqualVertexThreshold(float threshold) {
  m_equalVertexThreshold = threshold;
 }

 float getEqualVertexThreshold() {
  return m_equalVertexThreshold;
 }

 /**
  *
  * @param v
  * @return
  */
 @Override
 public boolean closest(final btVector3 v) {
  boolean succes = updateClosestVectorAndPoints();
  v.set(m_cachedV);
  return succes;
 }

 /**
  *
  * @return
  */
 @Override
 public float maxVertex() {
  int i, numverts = numVertices();
  float maxV = (0.f);
  for (i = 0; i < numverts; i++) {
   float curLen2 = m_simplexVectorW[i].lengthSquared();
   if (maxV < curLen2) {
    maxV = curLen2;
   }
  }
  return maxV;
 }

 /**
  *
  * @return
  */
 @Override
 public boolean fullSimplex() {
  return (m_numVertices == 4);
 }

 /**
  *
  * @param pBuf
  * @param qBuf
  * @param yBuf
  * @return
  */
 @Override
 public int getSimplex(btVector3[] pBuf, btVector3[] qBuf, btVector3[] yBuf) {
  int i;
  for (i = 0; i < numVertices(); i++) {
   yBuf[i].set(m_simplexVectorW[i]);
   pBuf[i].set(m_simplexPointsP[i]);
   qBuf[i].set(m_simplexPointsQ[i]);
  }
  return numVertices();
 }

 /**
  *
  * @param w
  * @return
  */
 @Override
 public boolean inSimplex(final btVector3 w) {
  boolean found = false;
  int i, numverts = numVertices();
  //float maxV = float(0.);
  //w is in the current (reduced) simplex
  for (i = 0; i < numverts; i++) {
   if (m_simplexVectorW[i].distanceSquared(w) <= m_equalVertexThreshold) {
    found = true;
    break;
   }
  }
  //check in case lastW is already removed
  if (w.equals(m_lastW)) {
   return true;
  }
  return found;
 }

 /**
  *
  * @param v
  */
 @Override
 public void backup_closest(final btVector3 v) {
  v.set(m_cachedV);
 }

 /**
  *
  * @return
  */
 @Override
 public boolean emptySimplex() {
  return (numVertices() == 0);
 }

 /**
  *
  * @param p1
  * @param p2
  */
 @Override
 public void compute_points(final btVector3 p1, final btVector3 p2) {
  updateClosestVectorAndPoints();
  p1.set(m_cachedP1);
  p2.set(m_cachedP2);
 }

 /**
  *
  * @return
  */
 @Override
 public int numVertices() {
  return m_numVertices;
 }
}
