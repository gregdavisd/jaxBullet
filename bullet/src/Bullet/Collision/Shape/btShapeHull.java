/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org
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
package Bullet.Collision.Shape;

import static Bullet.Collision.Shape.btConvexShape.MAX_PREFERRED_PENETRATION_DIRECTIONS;
import Bullet.LinearMath.Hull.HullDesc;
import static Bullet.LinearMath.Hull.HullError.QE_FAIL;
import static Bullet.LinearMath.Hull.HullFlag.QF_TRIANGLES;
import Bullet.LinearMath.Hull.HullLibrary;
import Bullet.LinearMath.Hull.HullResult;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;
import java.util.Arrays;
import java.util.Objects;

/**
 *
 * @author Gregery Barton
 */
public class btShapeHull implements Serializable {

 public static final int NUM_UNITSPHERE_POINTS = 42;
 public btVector3[] m_vertices;
 public int[] m_indices;
 public int m_numIndices;
 final btConvexShape m_shape;

 static btVector3[] getUnitSpherePoints() {
  return sUnitSpherePoints;
 }

 public btShapeHull(btConvexShape shape) {
  m_shape = shape;
 }

 public boolean buildHull(float margin) {
  int numSampleDirections = NUM_UNITSPHERE_POINTS;
  {
   int numPDA = m_shape.getNumPreferredPenetrationDirections();
   if (numPDA != 0) {
    final btVector3 norm = new btVector3();
    for (int i = 0; i < numPDA; i++) {
     m_shape.getPreferredPenetrationDirection(i, norm);
     getUnitSpherePoints()[numSampleDirections].set(norm);
     numSampleDirections++;
    }
   }
  }
  btVector3[] supportPoints = new btVector3[NUM_UNITSPHERE_POINTS
   + MAX_PREFERRED_PENETRATION_DIRECTIONS * 2];
  init(supportPoints);
  int i;
  for (i = 0; i < numSampleDirections; i++) {
   supportPoints[i].set(m_shape.localGetSupportingVertex(
    getUnitSpherePoints()[i]));
  }
  HullDesc hd = new HullDesc();
  hd.mFlags = QF_TRIANGLES;
  hd.mVcount = numSampleDirections;
  hd.mVertices = supportPoints;
  hd.mVertexStride = 1;
  HullLibrary hl = new HullLibrary();
  HullResult hr = new HullResult();
  if (hl.CreateConvexHull(hd, hr) == QE_FAIL) {
   return false;
  }
  m_vertices = hr.m_OutputVertices;
  m_numIndices = hr.mNumIndices;
  assert (hr.m_Indices.length == m_numIndices);
  m_indices = hr.m_Indices;
  // free temporary hull result that we just copied
  hl.ReleaseResult(hr);
  return true;
 }

 public int numTriangles() {
  return (m_numIndices / 3);
 }

 public int numVertices() {
  return m_vertices.length;
 }

 public int numIndices() {
  return m_numIndices;
 }

 public btVector3[] getVertexPointer() {
  return m_vertices;
 }

 public int[] getIndexPointer() {
  return m_indices;
 }

 static btVector3[] sUnitSpherePoints = new btVector3[]{
  new btVector3((0.000000f), (-0.000000f), (-1.000000f)),
  new btVector3((0.723608f), (-0.525725f), (-0.447219f)),
  new btVector3((-0.276388f), (-0.850649f), (-0.447219f)),
  new btVector3((-0.894426f), (-0.000000f), (-0.447216f)),
  new btVector3((-0.276388f), (0.850649f), (-0.447220f)),
  new btVector3((0.723608f), (0.525725f), (-0.447219f)),
  new btVector3((0.276388f), (-0.850649f), (0.447220f)),
  new btVector3((-0.723608f), (-0.525725f), (0.447219f)),
  new btVector3((-0.723608f), (0.525725f), (0.447219f)),
  new btVector3((0.276388f), (0.850649f), (0.447219f)),
  new btVector3((0.894426f), (0.000000f), (0.447216f)),
  new btVector3((-0.000000f), (0.000000f), (1.000000f)),
  new btVector3((0.425323f), (-0.309011f), (-0.850654f)),
  new btVector3((-0.162456f), (-0.499995f), (-0.850654f)),
  new btVector3((0.262869f), (-0.809012f), (-0.525738f)),
  new btVector3((0.425323f), (0.309011f), (-0.850654f)),
  new btVector3((0.850648f), (-0.000000f), (-0.525736f)),
  new btVector3((-0.525730f), (-0.000000f), (-0.850652f)),
  new btVector3((-0.688190f), (-0.499997f), (-0.525736f)),
  new btVector3((-0.162456f), (0.499995f), (-0.850654f)),
  new btVector3((-0.688190f), (0.499997f), (-0.525736f)),
  new btVector3((0.262869f), (0.809012f), (-0.525738f)),
  new btVector3((0.951058f), (0.309013f), (0.000000f)),
  new btVector3((0.951058f), (-0.309013f), (0.000000f)),
  new btVector3((0.587786f), (-0.809017f), (0.000000f)),
  new btVector3((0.000000f), (-1.000000f), (0.000000f)),
  new btVector3((-0.587786f), (-0.809017f), (0.000000f)),
  new btVector3((-0.951058f), (-0.309013f), (-0.000000f)),
  new btVector3((-0.951058f), (0.309013f), (-0.000000f)),
  new btVector3((-0.587786f), (0.809017f), (-0.000000f)),
  new btVector3((-0.000000f), (1.000000f), (-0.000000f)),
  new btVector3((0.587786f), (0.809017f), (-0.000000f)),
  new btVector3((0.688190f), (-0.499997f), (0.525736f)),
  new btVector3((-0.262869f), (-0.809012f), (0.525738f)),
  new btVector3((-0.850648f), (0.000000f), (0.525736f)),
  new btVector3((-0.262869f), (0.809012f), (0.525738f)),
  new btVector3((0.688190f), (0.499997f), (0.525736f)),
  new btVector3((0.525730f), (0.000000f), (0.850652f)),
  new btVector3((0.162456f), (-0.499995f), (0.850654f)),
  new btVector3((-0.425323f), (-0.309011f), (0.850654f)),
  new btVector3((-0.425323f), (0.309011f), (0.850654f)),
  new btVector3((0.162456f), (0.499995f), (0.850654f))
 };

 @Override
 public int hashCode() {
  int hash = 5;
  hash += super.hashCode();
  hash = 11 * hash + Arrays.deepHashCode(this.m_vertices);
  hash = 11 * hash + Arrays.hashCode(this.m_indices);
  hash = 11 * hash + this.m_numIndices;
  hash = 11 * hash + Objects.hashCode(this.m_shape);
  return hash;
 }

 @Override
 public boolean equals(Object obj) {
  if (this == obj) {
   return true;
  }
  if (obj == null) {
   return false;
  }
  if (getClass() != obj.getClass()) {
   return false;
  }
  final btShapeHull other = (btShapeHull) obj;
  if (this.m_numIndices != other.m_numIndices) {
   return false;
  }
  if (!Arrays.deepEquals(this.m_vertices, other.m_vertices)) {
   return false;
  }
  if (!Arrays.equals(this.m_indices, other.m_indices)) {
   return false;
  }
  if (!Objects.equals(this.m_shape, other.m_shape)) {
   return false;
  }
  return super.equals(obj);
 }

}
