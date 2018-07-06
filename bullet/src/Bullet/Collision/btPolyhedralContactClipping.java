/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it freely,
 * subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
///This file was written by Erwin Coumans
package Bullet.Collision;

import Bullet.Collision.Algorithm.Detector.btDiscreteCollisionDetectorInterface;
import static Bullet.LinearMath.btScalar.FLT_MAX;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btDot;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class btPolyhedralContactClipping implements Serializable {

 static int gActualSATPairTests;
 static int gExpectedNbTests;
 static int gActualNbTests;
 static boolean gUseInternalObject = true;

 public static void clipHullAgainstHull(final btVector3 separatingNormal1,
  btConvexPolyhedron hullA,
  btConvexPolyhedron hullB, final btTransform transA, final btTransform transB,
  float minDist,
  float maxDist,
  ArrayList<btVector3> worldVertsB1, ArrayList<btVector3> worldVertsB2,
  btDiscreteCollisionDetectorInterface.Result resultOut) {
  final btVector3 separatingNormal = new btVector3(separatingNormal1)
   .normalize();
  int closestFaceB = -1;
  float dmax = -FLT_MAX;
  {
   for (int face = 0; face < hullB.m_faces.size(); face++) {
    final btVector3 Normal = new btVector3(hullB.m_faces.get(face).m_plane[0],
     hullB.m_faces.get(face).m_plane[1], hullB.m_faces.get(face).m_plane[2]);
    final btVector3 WorldNormal = transB.transform3x3(new btVector3(Normal));
    float d = WorldNormal.dot(separatingNormal);
    if (d > dmax) {
     dmax = d;
     closestFaceB = face;
    }
   }
  }
  worldVertsB1.clear();
  {
   btFace polyB = hullB.m_faces.get(closestFaceB);
   int numVertices = polyB.m_indices.length;
   for (int e0 = 0; e0 < numVertices; e0++) {
    final btVector3 b = hullB.m_vertices.get(polyB.m_indices[e0]);
    worldVertsB1.add(transB.transform(new btVector3(b)));
   }
  }
  if (closestFaceB >= 0) {
   clipFaceAgainstHull(separatingNormal, hullA, transA, worldVertsB1,
    worldVertsB2, minDist, maxDist,
    resultOut);
  }
 }

 public static void clipFaceAgainstHull(final btVector3 separatingNormal,
  btConvexPolyhedron hullA,
  final btTransform transA, ArrayList<btVector3> worldVertsB1,
  ArrayList<btVector3> worldVertsB2,
  float minDist, float maxDist,
  btDiscreteCollisionDetectorInterface.Result resultOut) {
  worldVertsB2.clear();
  ArrayList<btVector3> pVtxIn = worldVertsB1;
  ArrayList<btVector3> pVtxOut = worldVertsB2;
  pVtxOut.ensureCapacity(pVtxIn.size());
  int closestFaceA = -1;
  {
   float dmin = FLT_MAX;
   for (int face = 0; face < hullA.m_faces.size(); face++) {
    final btVector3 Normal = new btVector3(hullA.m_faces.get(face).m_plane[0],
     hullA.m_faces.get(face).m_plane[1], hullA.m_faces.get(face).m_plane[2]);
    final btVector3 faceANormalWS = transA.transform3x3(new btVector3(Normal));
    float d = faceANormalWS.dot(separatingNormal);
    if (d < dmin) {
     dmin = d;
     closestFaceA = face;
    }
   }
  }
  if (closestFaceA < 0) {
   return;
  }
  btFace polyA = hullA.m_faces.get(closestFaceA);
  // clip polygon to back of planes of all faces of hull A that are adjacent to witness face
  int numVerticesA = polyA.m_indices.length;
  for (int e0 = 0; e0 < numVerticesA; e0++) {
   final btVector3 a = hullA.m_vertices.get(polyA.m_indices[e0]);
   final btVector3 b = hullA.m_vertices.get(polyA.m_indices[(e0 + 1)
    % numVerticesA]);
   final btVector3 edge0 = new btVector3(a).sub(b);
   final btVector3 WorldEdge0 = transA.transform3x3(new btVector3(edge0));
   final btVector3 worldPlaneAnormal1 = transA.transform3x3(new btVector3(
    polyA.m_plane[0],
    polyA.m_plane[1], polyA.m_plane[2]));
   final btVector3 planeNormalWS1 = new btVector3(WorldEdge0).negate().cross(
    worldPlaneAnormal1);//.cross(WorldEdge0);
   final btVector3 worldA1 = transA.transform3x3(a, new btVector3());
   float planeEqWS1 = -worldA1.dot(planeNormalWS1);
   final btVector3 planeNormalWS = new btVector3(planeNormalWS1);
   float planeEqWS = planeEqWS1;
   //clip face
   clipFace(pVtxIn, pVtxOut, planeNormalWS, planeEqWS);
   {
    ArrayList<btVector3> swapper = pVtxIn;
    pVtxIn = pVtxOut;
    pVtxOut = swapper;
   }
   pVtxOut.clear();
  }
//#define ONLY_REPORT_DEEPEST_POINT
// only keep points that are behind the witness face
  {
   final btVector3 localPlaneNormal = new btVector3(polyA.m_plane[0],
    polyA.m_plane[1],
    polyA.m_plane[2]);
   float localPlaneEq = polyA.m_plane[3];
   final btVector3 planeNormalWS = transA.transform3x3(new btVector3(
    localPlaneNormal));
   float planeEqWS = localPlaneEq - planeNormalWS.dot(transA.getOrigin());
   for (int i = 0; i < pVtxIn.size(); i++) {
    final btVector3 vtx = new btVector3(pVtxIn.get(i));
    float depth = planeNormalWS.dot(vtx) + planeEqWS;
    if (depth <= minDist) {
//				printf("clamped: depth=%f to minDist=%f\n",depth,minDist);
     depth = minDist;
    }
    if (depth <= maxDist) {
     final btVector3 point = new btVector3(pVtxIn.get(i));
     resultOut.addContactPoint(separatingNormal, point, depth);
    }
   }
  }
 }

 public static boolean findSeparatingAxis(btConvexPolyhedron hullA,
  btConvexPolyhedron hullB,
  final btTransform transA, final btTransform transB, final btVector3 sep,
  btDiscreteCollisionDetectorInterface.Result resultOut) {
  gActualSATPairTests++;
  final btVector3 c0 = transA.transform(new btVector3(hullA.m_localCenter));
  final btVector3 c1 = transB.transform(new btVector3(hullB.m_localCenter));
  final btVector3 DeltaC2 = new btVector3(c0).sub(c1);
  float dmin = FLT_MAX;
  int numFacesA = hullA.m_faces.size();
  // Test normals from hullA
  for (int i = 0; i < numFacesA; i++) {
   final btVector3 Normal
    = new btVector3(hullA.m_faces.get(i).m_plane[0],
     hullA.m_faces.get(i).m_plane[1], hullA.m_faces
     .get(i).m_plane[2]);
   final btVector3 faceANormalWS = transA.transform3x3(new btVector3(Normal));
   if (DeltaC2.dot(faceANormalWS) < 0) {
    faceANormalWS.negate();
   }
   gExpectedNbTests++;
   if (gUseInternalObject && !TestInternalObjects(transA, transB, DeltaC2,
    faceANormalWS, hullA,
    hullB, dmin)) {
    continue;
   }
   gActualNbTests++;
   float[] d = new float[1];
   final btVector3 wA = new btVector3();
   final btVector3 wB = new btVector3();
   if (!TestSepAxis(hullA, hullB, transA, transB, faceANormalWS, d, wA, wB)) {
    return false;
   }
   if (d[0] < dmin) {
    dmin = d[0];
    sep.set(faceANormalWS);
   }
  }
  int numFacesB = hullB.m_faces.size();
  // Test normals from hullB
  for (int i = 0; i < numFacesB; i++) {
   final btVector3 Normal
    = new btVector3(hullB.m_faces.get(i).m_plane[0],
     hullB.m_faces.get(i).m_plane[1], hullB.m_faces
     .get(i).m_plane[2]);
   final btVector3 WorldNormal = transB.transform3x3(new btVector3(Normal));
   if (DeltaC2.dot(WorldNormal) < 0) {
    WorldNormal.negate();
   }
   gExpectedNbTests++;
   if (gUseInternalObject && !TestInternalObjects(transA, transB, DeltaC2,
    WorldNormal, hullA, hullB, dmin)) {
    continue;
   }
   gActualNbTests++;
   float[] d = new float[1];
   final btVector3 wA = new btVector3();
   final btVector3 wB = new btVector3();
   if (!TestSepAxis(hullA, hullB, transA, transB, WorldNormal, d, wA, wB)) {
    return false;
   }
   if (d[0] < dmin) {
    dmin = d[0];
    sep.set(WorldNormal);
   }
  }
  //final btVector3 edgeAstart=new btVector3();final btVector3 edgeAend=new btVector3();final btVector3 edgeBstart=new btVector3();final btVector3 edgeBend=new btVector3();
  int edgeA = -1;
  int edgeB = -1;
  final btVector3 worldEdgeA = new btVector3();
  final btVector3 worldEdgeB = new btVector3();
  final btVector3 witnessPointA = new btVector3();
  final btVector3 witnessPointB = new btVector3();
  // Test edges
  for (int e0 = 0; e0 < hullA.m_uniqueEdges.size(); e0++) {
   final btVector3 edge0 = new btVector3(hullA.m_uniqueEdges.get(e0));
   final btVector3 WorldEdge0 = transA.transform3x3(new btVector3(edge0));
   for (int e1 = 0; e1 < hullB.m_uniqueEdges.size(); e1++) {
    final btVector3 edge1 = new btVector3(hullB.m_uniqueEdges.get(e1));
    final btVector3 WorldEdge1 = transB.transform3x3(new btVector3(edge1));
    final btVector3 Cross = new btVector3(WorldEdge0).cross(WorldEdge1);
    if (!IsAlmostZero(Cross)) {
     Cross.normalize();
     if (DeltaC2.dot(Cross) < 0) {
      Cross.negate();
     }
     gExpectedNbTests++;
     if (gUseInternalObject && !TestInternalObjects(transA, transB, DeltaC2,
      Cross, hullA, hullB,
      dmin)) {
      continue;
     }
     gActualNbTests++;
     float[] dist = new float[1];
     final btVector3 wA = new btVector3();
     final btVector3 wB = new btVector3();
     if (!TestSepAxis(hullA, hullB, transA, transB, Cross, dist, wA, wB)) {
      return false;
     }
     if (dist[0] < dmin) {
      dmin = dist[0];
      sep.set(Cross);
      edgeA = e0;
      edgeB = e1;
      worldEdgeA.set(WorldEdge0);
      worldEdgeB.set(WorldEdge1);
      witnessPointA.set(wA);
      witnessPointB.set(wB);
     }
    }
   }
  }
  if (edgeA >= 0 && edgeB >= 0) {
//		printf("edge-edge\n");
   //add an edge-edge contact
   final btVector3 ptsVector = new btVector3();
   final btVector3 offsetA = new btVector3();
   final btVector3 offsetB = new btVector3();
   float[] tA = new float[1];
   float[] tB = new float[1];
   final btVector3 translation = new btVector3(witnessPointB).sub(witnessPointA);
   final btVector3 dirA = new btVector3(worldEdgeA);
   final btVector3 dirB = new btVector3(worldEdgeB);
   float hlenB = 1e30f;
   float hlenA = 1e30f;
   btSegmentsClosestPoints(ptsVector, offsetA, offsetB, tA, tB,
    translation,
    dirA, hlenA,
    dirB, hlenB);
   float nlSqrt = ptsVector.lengthSquared();
   if (nlSqrt > SIMD_EPSILON) {
    float nl = btSqrt(nlSqrt);
    ptsVector.scale(1.f / nl);
    if (ptsVector.dot(DeltaC2) < 0.f) {
     ptsVector.negate();
    }
    final btVector3 ptOnB = new btVector3(witnessPointB).add(offsetB);
    float distance = nl;
    resultOut.addContactPoint(ptsVector, ptOnB, -distance);
   }
  }
  if ((DeltaC2.dot(sep)) < 0.0f) {
   sep.negate();
  }
  return true;
 }

 ///the clipFace method is used internally
 static void clipFace(ArrayList<btVector3> pVtxIn, ArrayList<btVector3> ppVtxOut,
  final btVector3 planeNormalWS, float planeEqWS) {
  int ve;
  float ds, de;
  int numVerts = pVtxIn.size();
  if (numVerts < 2) {
   return;
  }
  final btVector3 firstVertex = new btVector3(pVtxIn.get(pVtxIn.size() - 1));
  final btVector3 endVertex = new btVector3(pVtxIn.get(0));
  ds = planeNormalWS.dot(firstVertex) + planeEqWS;
  for (ve = 0; ve < numVerts; ve++) {
   endVertex.set(pVtxIn.get(ve));
   de = planeNormalWS.dot(endVertex) + planeEqWS;
   if (ds < 0) {
    if (de < 0) {
     // Start < 0, end < 0, so output endVertex
     ppVtxOut.add(endVertex);
    } else {
     // Start < 0, end >= 0, so output intersection
     ppVtxOut.add(new btVector3(firstVertex).interpolate(endVertex, (ds * 1.f
      / (ds - de))));
    }
   } else if (de < 0) {
    // Start >= 0, end < 0 so output intersection and end
    ppVtxOut.add(new btVector3(firstVertex).interpolate(endVertex, (ds * 1.f
     / (ds - de))));
    ppVtxOut.add(new btVector3(endVertex));
   }
   firstVertex.set(endVertex);
   ds = de;
  }
 }

 static boolean TestInternalObjects(final btTransform trans0,
  final btTransform trans1,
  final btVector3 delta_c, final btVector3 axis, btConvexPolyhedron convex0,
  btConvexPolyhedron convex1, float dmin) {
  float dp = delta_c.dot(axis);
  final btVector3 localAxis0 = new btVector3();
  InverseTransformPoint3x3(localAxis0, axis, trans0);
  final btVector3 localAxis1 = new btVector3();
  InverseTransformPoint3x3(localAxis1, axis, trans1);
  float p0[] = new float[3];
  BoxSupport(convex0.m_extents, localAxis0, p0);
  float p1[] = new float[3];
  BoxSupport(convex1.m_extents, localAxis1, p1);
  float Radius0 = p0[0] * localAxis0.x() + p0[1] * localAxis0.y() + p0[2]
   * localAxis0.z();
  float Radius1 = p1[0] * localAxis1.x() + p1[1] * localAxis1.y() + p1[2]
   * localAxis1.z();
  float MinRadius = Radius0 > convex0.m_radius ? Radius0 : convex0.m_radius;
  float MaxRadius = Radius1 > convex1.m_radius ? Radius1 : convex1.m_radius;
  float MinMaxRadius = MaxRadius + MinRadius;
  float d0 = MinMaxRadius + dp;
  float d1 = MinMaxRadius - dp;
  float depth = d0 < d1 ? d0 : d1;
  return depth <= dmin;
 }

 static void InverseTransformPoint3x3(final btVector3 out, final btVector3 in,
  final btTransform tr) {
  float x = tr.m00 * in.x() + tr.m10 * in.y() + tr.m20 * in.z();
  float y = tr.m01 * in.x() + tr.m11 * in.y() + tr.m21 * in.z();
  float z = tr.m02 * in.x() + tr.m12 * in.y() + tr.m22 * in.z();
  out.set(x, y, z);
 }

 static void BoxSupport(final btVector3 extents, final btVector3 sv, float[] p) {
  p[0] = sv.x < 0.0f ? -extents.x : extents.x;
  p[1] = sv.y < 0.0f ? -extents.y : extents.y;
  p[2] = sv.z < 0.0f ? -extents.z : extents.z;
 }

 static boolean TestSepAxis(btConvexPolyhedron hullA, btConvexPolyhedron hullB,
  final btTransform transA, final btTransform transB, final btVector3 sep_axis,
  float[] depth,
  final btVector3 witnessPointA, final btVector3 witnessPointB) {
  float[] Min0 = new float[1];
  float[] Max0 = new float[1];
  float[] Min1 = new float[1];
  float[] Max1 = new float[2];
  final btVector3 witnesPtMinA = new btVector3();
  final btVector3 witnesPtMaxA = new btVector3();
  final btVector3 witnesPtMinB = new btVector3();
  final btVector3 witnesPtMaxB = new btVector3();
  hullA.project(transA, sep_axis, Min0, Max0, witnesPtMinA, witnesPtMaxA);
  hullB.project(transB, sep_axis, Min1, Max1, witnesPtMinB, witnesPtMaxB);
  if (Max0[0] < Min1[0] || Max1[0] < Min0[0]) {
   return false;
  }
  float d0 = Max0[0] - Min1[0];
  assert (d0 >= 0.0f);
  float d1 = Max1[0] - Min0[0];
  assert (d1 >= 0.0f);
  if (d0 < d1) {
   depth[0] = d0;
   witnessPointA.set(witnesPtMaxA);
   witnessPointB.set(witnesPtMinB);
  } else {
   depth[0] = d1;
   witnessPointA.set(witnesPtMinA);
   witnessPointB.set(witnesPtMaxB);
  }
  return true;
 }

 static boolean IsAlmostZero(final btVector3 v) {
  return !(btFabs(v.x()) > 1e-6 || btFabs(v.y()) > 1e-6 || btFabs(v.z()) > 1e-6);
 }

 static void btSegmentsClosestPoints(
  final btVector3 ptsVector, final btVector3 offsetA, final btVector3 offsetB,
  float[] tA, float[] tB, final btVector3 translation, final btVector3 dirA,
  float hlenA,
  final btVector3 dirB, float hlenB) {
  // compute the parameters of the closest points on each line segment
  float dirA_dot_dirB = btDot(dirA, dirB);
  float dirA_dot_trans = btDot(dirA, translation);
  float dirB_dot_trans = btDot(dirB, translation);
  float denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;
  if (denom == 0.0f) {
   tA[0] = 0.0f;
  } else {
   tA[0] = (dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB) / denom;
   if (tA[0] < -hlenA) {
    tA[0] = -hlenA;
   } else if (tA[0] > hlenA) {
    tA[0] = hlenA;
   }
  }
  tB[0] = tA[0] * dirA_dot_dirB - dirB_dot_trans;
  if (tB[0] < -hlenB) {
   tB[0] = -hlenB;
   tA[0] = tB[0] * dirA_dot_dirB + dirA_dot_trans;
   if (tA[0] < -hlenA) {
    tA[0] = -hlenA;
   } else if (tA[0] > hlenA) {
    tA[0] = hlenA;
   }
  } else if (tB[0] > hlenB) {
   tB[0] = hlenB;
   tA[0] = tB[0] * dirA_dot_dirB + dirA_dot_trans;
   if (tA[0] < -hlenA) {
    tA[0] = -hlenA;
   } else if (tA[0] > hlenA) {
    tA[0] = hlenA;
   }
  }
  // compute the closest points relative to segment centers.
  offsetA.set(new btVector3(dirA).scale(tA[0]));
  offsetB.set(new btVector3(dirB).scale(tB[0]));
  ptsVector.set(translation).sub(offsetA).add(offsetB);
 }

}
