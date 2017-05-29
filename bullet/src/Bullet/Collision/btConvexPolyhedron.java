/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
///This file was written by Erwin Coumans
///Separating axis rest based on work from Pierre Terdiman, see
///And contact clipping based on work from Simon Hobbs
package Bullet.Collision;

import static Bullet.LinearMath.btScalar.FLT_MAX;
import static Bullet.LinearMath.btScalar.btFabs;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

/**
 *
 * @author Gregery Barton
 */
public class btConvexPolyhedron implements Serializable {

 public final ArrayList<btVector3> m_vertices = new ArrayList<>(0);
 public final ArrayList<btFace> m_faces = new ArrayList<>(0);
 public final ArrayList<btVector3> m_uniqueEdges = new ArrayList<>(0);
 public final btVector3 m_localCenter = new btVector3();
 public final btVector3 m_extents = new btVector3();
 public float m_radius;
 public final btVector3 mC = new btVector3();
 public final btVector3 mE = new btVector3();

 public btConvexPolyhedron() {
 }

 public final void initialize() {
  final HashMap<btInternalVertexPair, btInternalEdge> edges = new HashMap<>();
  float TotalArea = 0.0f;
  m_localCenter.set(0, 0, 0);
  for (int i = 0; i < m_faces.size(); i++) {
   int numVertices = m_faces.get(i).m_indices.length;
   int NbTris = numVertices;
   for (int j = 0; j < NbTris; j++) {
    int k = (j + 1) % numVertices;
    btInternalVertexPair vp = new btInternalVertexPair(m_faces.get(i).m_indices[j], m_faces.get(
     i).m_indices[k]);
    btInternalEdge edptr = edges.get(vp);
    final btVector3 edge = new btVector3(m_vertices.get(vp.m_v1)).sub(m_vertices.get(vp.m_v0));
    edge.normalize();
    boolean found = false;
    for (int p = 0; p < m_uniqueEdges.size(); p++) {
     if (IsAlmostZero(new btVector3(m_uniqueEdges.get(p)).sub(edge)) ||
      IsAlmostZero(new btVector3(m_uniqueEdges.get(p)).add(edge))) {
      found = true;
      break;
     }
    }
    if (!found) {
     m_uniqueEdges.add(edge);
    }
    if (edptr != null) {
     assert (edptr.m_face0 >= 0);
     assert (edptr.m_face1 < 0);
     edptr.m_face1 = i;
    } else {
     btInternalEdge ed = new btInternalEdge();
     ed.m_face0 = i;
     edges.put(vp, ed);
    }
   }
  }
  final btVector3 a = new btVector3();
  final btVector3 b = new btVector3();
  final btVector3 Center = new btVector3();
  for (int i = 0; i < m_faces.size(); i++) {
   btFace face = m_faces.get(i);
   int numVertices = face.m_indices.length;
   int NbTris = numVertices - 2;
   final btVector3 p0 = m_vertices.get(face.m_indices[0]);
   for (int j = 1; j <= NbTris; j++) {
    int k = (j + 1) % numVertices;
    final btVector3 p1 = m_vertices.get(face.m_indices[j]);
    final btVector3 p2 = m_vertices.get(face.m_indices[k]);
    a.set(p0).sub(p1);
    b.set(p0).sub(p2);
    float Area = a.cross(b).length() * 0.5f;
//						float Area = ((p0 - p1).cross(p0 - p2)).length() * 0.5f;
    Center.set(p0).add(p1).add(p2).scale(1.0f / 3.0f);
    m_localCenter.add(Center.scale(Area));
    TotalArea += Area;
   }
  }
  m_localCenter.scale(1.0f / TotalArea);
  if (true) {
   m_radius = FLT_MAX;
   {
    final btVector3 Normal = new btVector3();
    for (int i = 0; i < m_faces.size(); i++) {
     btFace face = m_faces.get(i);
     Normal.set(face.m_plane[0], face.m_plane[1], face.m_plane[2]);
     float dist = btFabs(m_localCenter.dot(Normal) + face.m_plane[3]);
     if (dist < m_radius) {
      m_radius = dist;
     }
    }
   }
   float MinX = FLT_MAX;
   float MinY = FLT_MAX;
   float MinZ = FLT_MAX;
   float MaxX = -FLT_MAX;
   float MaxY = -FLT_MAX;
   float MaxZ = -FLT_MAX;
   for (int i = 0; i < m_vertices.size(); i++) {
    final btVector3 pt = m_vertices.get(i);
    if (pt.x() < MinX) {
     MinX = pt.x();
    }
    if (pt.x() > MaxX) {
     MaxX = pt.x();
    }
    if (pt.y() < MinY) {
     MinY = pt.y();
    }
    if (pt.y() > MaxY) {
     MaxY = pt.y();
    }
    if (pt.z() < MinZ) {
     MinZ = pt.z();
    }
    if (pt.z() > MaxZ) {
     MaxZ = pt.z();
    }
   }
   mC.set(MaxX + MinX, MaxY + MinY, MaxZ + MinZ);
   mE.set(MaxX - MinX, MaxY - MinY, MaxZ - MinZ);
//		  float r = m_radius / sqrtf(2.0f);
   float r = m_radius / btSqrt(3.0f);
   int LargestExtent = mE.maxAxis();
   float Step = (mE.getElement(LargestExtent) * 0.5f - r) / 1024.0f;
   m_extents.x = m_extents.y = m_extents.z = r;
   m_extents.setElement(LargestExtent, mE.getElement(LargestExtent) * 0.5f);
   boolean FoundBox = false;
   for (int j = 0; j < 1024; j++) {
    if (testContainment()) {
     FoundBox = true;
     break;
    }
    m_extents.setElement(LargestExtent, m_extents.getElement(LargestExtent) - Step);
   }
   if (!FoundBox) {
    m_extents.x = m_extents.y = m_extents.z = r;
   } else {
    // Refine the box
    Step = (m_radius - r) / 1024.0f;
    int e0 = (1 << LargestExtent) & 3;
    int e1 = (1 << e0) & 3;
    for (int j = 0; j < 1024; j++) {
     float Saved0 = m_extents.getElement(e0);
     float Saved1 = m_extents.getElement(e1);
     m_extents.setElement(e0, m_extents.getElement(e0) + Step);
     m_extents.setElement(e1, m_extents.getElement(e1) + Step);
     if (!testContainment()) {
      m_extents.setElement(e0, Saved0);
      m_extents.setElement(e1, Saved1);
      break;
     }
    }
   }
  }
 }

 boolean testContainment() {
  for (int p = 0; p < 8; p++) {
   final btVector3 LocalPt = new btVector3(m_localCenter);
   switch (p) {
    case 0:
     LocalPt.add(new btVector3(m_extents.x, m_extents.y, m_extents.z));
     break;
    case 1:
     LocalPt.add(new btVector3(m_extents.x, m_extents.y, -m_extents.z));
     break;
    case 2:
     LocalPt.add(new btVector3(m_extents.x, -m_extents.y, m_extents.z));
     break;
    case 3:
     LocalPt.add(new btVector3(m_extents.x, -m_extents.y, -m_extents.z));
     break;
    case 4:
     LocalPt.add(new btVector3(-m_extents.x, m_extents.y, m_extents.z));
     break;
    case 5:
     LocalPt.add(new btVector3(-m_extents.x, m_extents.y, -m_extents.z));
     break;
    case 6:
     LocalPt.add(new btVector3(-m_extents.x, -m_extents.y, m_extents.z));
     break;
    case 7:
     LocalPt.add(new btVector3(-m_extents.x, -m_extents.y, -m_extents.z));
     break;
    default:
     break;
   }
   final btVector3 Normal = new btVector3();
   for (int i = 0; i < m_faces.size(); i++) {
    btFace face = m_faces.get(i);
    Normal.set(face.m_plane[0], face.m_plane[1], face.m_plane[2]);
    float d = LocalPt.dot(Normal) + face.m_plane[3];
    if (d > 0.0f) {
     return false;
    }
   }
  }
  return true;
 }

 public void project(final btTransform trans, final btVector3 dir, float[] minProj, float[] maxProj,
  final btVector3 witnesPtMin, final btVector3 witnesPtMax) {
  minProj[0] = FLT_MAX;
  maxProj[0] = -FLT_MAX;
  int numVerts = m_vertices.size();
  final btVector3 pt = new btVector3();
  for (int i = 0; i < numVerts; i++) {
   pt.set(m_vertices.get(i));
   trans.transform(pt);
   float dp = pt.dot(dir);
   if (dp < minProj[0]) {
    minProj[0] = dp;
    witnesPtMin.set(pt);
   }
   if (dp > maxProj[0]) {
    maxProj[0] = dp;
    witnesPtMax.set(pt);
   }
  }
  if (minProj[0] > maxProj[0]) {
   {
    float swapper = minProj[0];
    minProj[0] = maxProj[0];
    maxProj[0] = swapper;
   }
   {
    final btVector3 swapper = new btVector3(witnesPtMin);
    witnesPtMin.set(witnesPtMax);
    witnesPtMax.set(swapper);
   }
  }
 }

 private boolean IsAlmostZero(final btVector3 v) {
  return !(btFabs(v.x()) > 1e-6 || btFabs(v.y()) > 1e-6 || btFabs(v.z()) > 1e-6);
 }
};
