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

import Bullet.Collision.btConvexPolyhedron;
import Bullet.Collision.btFace;
import static Bullet.Extras.btMinMax.btMin;
import Bullet.LinearMath.GrahamVector3;
import static Bullet.LinearMath.GrahamVector3.GrahamScanConvexHull2D;
import Bullet.LinearMath.btConvexHullComputer;
import Bullet.LinearMath.btGeometryUtil;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import Bullet.LinearMath.btVector4;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * @author Gregery Barton
 */
public abstract class btPolyhedralConvexShape extends btConvexInternalShape implements Serializable {

 btConvexPolyhedron m_polyhedron;

 public btPolyhedralConvexShape() {
  super();
 }

 ///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
 ///experimental/work-in-progress
 public boolean initializePolyhedralFeatures(int shiftVerticesByMargin) {
  if (false) {
   if (m_polyhedron != null) {
    m_polyhedron = null;
   }
   m_polyhedron = new btConvexPolyhedron();
   ArrayList<btVector3> orgVertices = new ArrayList<>(getNumVertices());
   for (int i = 0; i < getNumVertices(); i++) {
    orgVertices.add(new btVector3());
    getVertex(i, orgVertices.get(i));
   }
   btConvexHullComputer conv = new btConvexHullComputer();
   if (shiftVerticesByMargin != 0) {
    ArrayList<btVector4> planeEquations = new ArrayList<>();
    btGeometryUtil.getPlaneEquationsFromVertices(orgVertices, planeEquations);
    ArrayList<btVector4> shiftedPlaneEquations = new ArrayList<>(planeEquations.size());
    for (int p = 0; p < planeEquations.size(); p++) {
     btVector4 plane = new btVector4(planeEquations.get(p));
     plane.w -= getMargin();
     shiftedPlaneEquations.add(plane);
    }
    ArrayList<btVector3> tmpVertices = new ArrayList<>(shiftedPlaneEquations.size());
    btGeometryUtil.getVerticesFromPlaneEquations(shiftedPlaneEquations, tmpVertices);
    conv.compute(tmpVertices, tmpVertices.size(), 0.f, 0.f);
   } else {
    conv.compute(orgVertices, orgVertices.size(), 0.f, 0.f);
   }
   int numFaces = conv.faces.size();
   ArrayList<btVector3> faceNormals = new ArrayList<>(numFaces);
   btConvexHullComputer convexUtil = conv;
   ArrayList<btFace> tmpFaces = new ArrayList<>(numFaces);
   int numVertices = convexUtil.vertices.size();
   m_polyhedron.m_vertices.ensureCapacity(numVertices);
   for (int p = 0; p < numVertices; p++) {
    m_polyhedron.m_vertices.add(new btVector3(convexUtil.vertices.get(p)));
   }
   for (int i = 0; i < numFaces; i++) {
    int face = convexUtil.faces.get(i);
    //printf("face=%d\n",face);
    btConvexHullComputer.Edge firstEdge = convexUtil.edges.get(face);
    btConvexHullComputer.Edge edge = firstEdge;
    btVector3[] edges = new btVector3[3];
    init(edges);
    int numEdges = 0;
    //compute face normals
    do {
     int src = edge.getSourceVertex();
     tmpFaces.get(i).m_indices = Arrays.copyOfRange(tmpFaces.get(i).m_indices, 0,
      tmpFaces.get(i).m_indices.length + 1);
     int targ = edge.getTargetVertex();
     final btVector3 wa = new btVector3(convexUtil.vertices.get(src));
     final btVector3 wb = new btVector3(convexUtil.vertices.get(targ));
     final btVector3 newEdge = new btVector3(wb).sub(wa);
     newEdge.normalize();
     if (numEdges < 2) {
      edges[numEdges++] = newEdge;
     }
     edge = edge.getNextEdgeOfFace();
    } while (edge != firstEdge);
    float planeEq = 1e30f;
    if (numEdges == 2) {
     faceNormals.set(i, new btVector3(edges[0]).cross(edges[1]).normalize());
     tmpFaces.get(i).m_plane[0] = faceNormals.get(i).getX();
     tmpFaces.get(i).m_plane[1] = faceNormals.get(i).getY();
     tmpFaces.get(i).m_plane[2] = faceNormals.get(i).getZ();
     tmpFaces.get(i).m_plane[3] = planeEq;
    } else {
     assert (false);//degenerate?
     faceNormals.get(i).setZero();
    }
    for (int v = 0; v < tmpFaces.get(i).m_indices.length; v++) {
     float eq = m_polyhedron.m_vertices.get(tmpFaces.get(i).m_indices[v]).dot(faceNormals.get(i));
     if (planeEq > eq) {
      planeEq = eq;
     }
    }
    tmpFaces.get(i).m_plane[3] = -planeEq;
   }
   //merge coplanar faces and copy them to m_polyhedron
   float faceWeldThreshold = 0.999f;
   ArrayIntList todoFaces = new ArrayIntList(tmpFaces.size());
   for (int i = 0; i < tmpFaces.size(); i++) {
    todoFaces.add(i);
   }
   while (todoFaces.size() > 0) {
    ArrayIntList coplanarFaceGroup = new ArrayIntList();
    int refFace = todoFaces.get(todoFaces.size() - 1);
    coplanarFaceGroup.add(refFace);
    btFace faceA = tmpFaces.get(refFace);
    todoFaces.removeElementAt(todoFaces.size() - 1);
    final btVector3 faceNormalA =
     new btVector3(faceA.m_plane[0], faceA.m_plane[1], faceA.m_plane[2]);
    for (int j = todoFaces.size() - 1; j >= 0; j--) {
     int i = todoFaces.get(j);
     btFace faceB = tmpFaces.get(i);
     final btVector3 faceNormalB =
      new btVector3(faceB.m_plane[0], faceB.m_plane[1], faceB.m_plane[2]);
     if (faceNormalA.dot(faceNormalB) > faceWeldThreshold) {
      coplanarFaceGroup.add(i);
      todoFaces.removeElementAt(i);
     }
    }
    boolean did_merge = false;
    if (coplanarFaceGroup.size() > 1) {
     //do the merge: use Graham Scan 2d convex hull
     ArrayList<GrahamVector3> orgpoints = new ArrayList<>();
     final btVector3 averageFaceNormal = new btVector3();
     for (int i = 0; i < coplanarFaceGroup.size(); i++) {
//				m_polyhedron.m_faces.push_back(tmpFaces[coplanarFaceGroup[i]]);
      btFace face = tmpFaces.get(coplanarFaceGroup.get(i));
      final btVector3 faceNormal = new btVector3(face.m_plane[0], face.m_plane[1], face.m_plane[2]);
      averageFaceNormal.add(faceNormal);
      for (int f = 0; f < face.m_indices.length; f++) {
       int orgIndex = face.m_indices[f];
       final btVector3 pt = new btVector3(m_polyhedron.m_vertices.get(orgIndex));
       boolean found = false;
       for (i = 0; i < orgpoints.size(); i++) {
        //if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).lengthSquared()<0.0001))
        if (orgpoints.get(i).m_orgIndex == orgIndex) {
         found = true;
         break;
        }
       }
       if (!found) {
        orgpoints.add(new GrahamVector3(pt, orgIndex));
       }
      }
     }
     btFace combinedFace = new btFace();
     for (int i = 0; i < 4; i++) {
      combinedFace.m_plane[i] = tmpFaces.get(coplanarFaceGroup.get(0)).m_plane[i];
     }
     ArrayList<GrahamVector3> hull = new ArrayList<>();
     averageFaceNormal.normalize();
     GrahamScanConvexHull2D(orgpoints, hull, averageFaceNormal);
     for (int i = 0; i < hull.size(); i++) {
      combinedFace.m_indices = Arrays.copyOfRange(combinedFace.m_indices, 0,
       combinedFace.m_indices.length + 1);
      combinedFace.m_indices[combinedFace.m_indices.length - 1] = (hull.get(i).m_orgIndex);
      for (int k = 0; k < orgpoints.size(); k++) {
       if (orgpoints.get(k).m_orgIndex == hull.get(i).m_orgIndex) {
        orgpoints.get(k).m_orgIndex = -1; // invalidate...
        break;
       }
      }
     }
     // are there rejected vertices?
     boolean reject_merge = false;
     for (int i = 0; i < orgpoints.size(); i++) {
      if (orgpoints.get(i).m_orgIndex == -1) {
       continue; // this is in the hull...
      }				// this vertex is rejected -- is anybody else using this vertex?
      for (int j = 0; j < tmpFaces.size(); j++) {
       btFace face = tmpFaces.get(j);
       // is this a face of the current coplanar group?
       boolean is_in_current_group = false;
       for (int k = 0; k < coplanarFaceGroup.size(); k++) {
        if (coplanarFaceGroup.get(k) == j) {
         is_in_current_group = true;
         break;
        }
       }
       if (is_in_current_group) // ignore this face...
       {
        continue;
       }
       // does this face use this rejected vertex?
       for (int v = 0; v < face.m_indices.length; v++) {
        if (face.m_indices[v] == orgpoints.get(i).m_orgIndex) {
         // this rejected vertex is used in another face -- reject merge
         reject_merge = true;
         break;
        }
       }
       if (reject_merge) {
        break;
       }
      }
      if (reject_merge) {
       break;
      }
     }
     if (!reject_merge) {
      // do this merge!
      did_merge = true;
      m_polyhedron.m_faces.add(new btFace(combinedFace));
     }
    }
    if (!did_merge) {
     for (int i = 0; i < coplanarFaceGroup.size(); i++) {
      btFace face = new btFace(tmpFaces.get(coplanarFaceGroup.get(i)));
      m_polyhedron.m_faces.add(face);
     }
    }
   }
   m_polyhedron.initialize();
  }
  return true;
 }

 public boolean initializePolyhedralFeatures() {
  return initializePolyhedralFeatures(0);
 }

 public btConvexPolyhedron getConvexPolyhedron() {
  return m_polyhedron;
 }

 //brute force implementations
 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec0) {
  final btVector3 supVec = new btVector3();
  int i;
  float maxDot = (-BT_LARGE_FLOAT);
  final btVector3 vec = new btVector3(vec0);
  float lenSqr = vec.lengthSquared();
  if (lenSqr < (0.0001f)) {
   vec.set(1, 0, 0);
  } else {
   float rlen = (1.f) / btSqrt(lenSqr);
   vec.scale(rlen);
  }
  int numVertices = getNumVertices();
  //btVector3 vtx= new btVector3();
  float[] newDot = new float[1];
  btVector3[] temp = new btVector3[btMin(numVertices, 128)];
  init(temp);
  for (int k = 0; k < numVertices; k += 128) {
   int inner_count = btMin(numVertices - k, 128);
   for (i = 0; i < inner_count; i++) {
    getVertex(i, temp[i]);
   }
   i = vec.maxDot(temp, inner_count, newDot);
   if (newDot[0] > maxDot) {
    maxDot = newDot[0];
    supVec.set(temp[i]);
   }
  }
  return supVec;
 }

 public void batchedUnitVectorGetSupportingVertexWithoutMargin(btVector3[] vectors,
  btVector4[] supportVerticesOut, int numVectors) {
  int i;
  for (i = 0; i < numVectors; i++) {
   if (supportVerticesOut[i] == null) {
    supportVerticesOut[i] = new btVector4();
   }
   supportVerticesOut[i].w = -BT_LARGE_FLOAT;
  }
  float[] newDot = new float[1];
  int numVertices = getNumVertices();
  btVector3[] temp = new btVector3[btMin(128, numVertices)];
  for (int j = 0; j < numVectors; j++) {
   final btVector3 vec = vectors[j];
   for (int k = 0; k < numVertices; k += 128) {
    int inner_count = btMin(numVertices - k, 128);
    for (i = 0; i < inner_count; i++) {
     getVertex(i, temp[i]);
    }
    i = vec.maxDot(temp, inner_count, newDot);
    if (newDot[0] > supportVerticesOut[j].w) {
     supportVerticesOut[j].set(temp[i]).w = newDot[0];
    }
   }
  }
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //not yet, return box inertia
  float margin = getMargin();
  final btTransform ident = new btTransform();
  ident.setIdentity();
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  getAabb(ident, aabbMin, aabbMax);
  final btVector3 halfExtents = new btVector3(aabbMax).sub(aabbMin).scale(0.5f);
  float lx = (2.f) * (halfExtents.x() + margin);
  float ly = (2.f) * (halfExtents.y() + margin);
  float lz = (2.f) * (halfExtents.z() + margin);
  float x2 = lx * lx;
  float y2 = ly * ly;
  float z2 = lz * lz;
  float scaledmass = mass * (0.08333333f);
  inertia.set(new btVector3(y2 + z2, x2 + z2, x2 + y2).scale(scaledmass));
 }

 public abstract int getNumVertices();

 public abstract int getNumEdges();

 public abstract void getEdge(int i, final btVector3 pa, final btVector3 pb);

 public abstract void getVertex(int i, final btVector3 vtx);

 public abstract int getNumPlanes();

 public abstract void getPlane(final btVector3 planeNormal, final btVector3 planeSupport, int i);

 public abstract boolean isInside(final btVector3 pt, float tolerance);
};
