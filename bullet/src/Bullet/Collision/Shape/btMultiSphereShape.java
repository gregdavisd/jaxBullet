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

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.MULTI_SPHERE_SHAPE_PROXYTYPE;
import static Bullet.Extras.btMinMax.btMin;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.init;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Objects;
import org.apache.commons.collections.primitives.ArrayFloatList;

/**
 *
 * @author Gregery Barton
 */
public class btMultiSphereShape extends btConvexInternalAabbCachingShape
 implements Serializable {

 final ArrayList<btVector3> m_localPositionArray = new ArrayList<>(0);
 final ArrayFloatList m_radiArray = new ArrayFloatList();

 btMultiSphereShape(btVector3[] positions, float[] radi, int numSpheres) {
  super();
  m_shapeType = MULTI_SPHERE_SHAPE_PROXYTYPE;
  //float startMargin = float(BT_LARGE_FLOAT);
  m_localPositionArray.ensureCapacity(numSpheres);
  m_radiArray.ensureCapacity(numSpheres);
  for (int i = 0; i < numSpheres; i++) {
   m_localPositionArray.add(new btVector3(positions[i]));
   m_radiArray.add(radi[i]);
  }
  recalcLocalAabb();
 }

 ///CollisionShape Interface
 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //as an approximation, take the inertia of the box that bounds the spheres
  final btVector3 localAabbMin = new btVector3();
  final btVector3 localAabbMax = new btVector3();
  getCachedLocalAabb(localAabbMin, localAabbMax);
  final btVector3 halfExtents = new btVector3(localAabbMax).sub(localAabbMin)
   .scale(0.5f);
  float lx = (2.f) * (halfExtents.x());
  float ly = (2.f) * (halfExtents.y());
  float lz = (2.f) * (halfExtents.z());
  inertia.set(mass / ((12.0f)) * (ly * ly + lz * lz),
   mass / ((12.0f)) * (lx * lx + lz * lz),
   mass / ((12.0f)) * (lx * lx + ly * ly));
 }

 /// btConvexShape Interface
 @Override
 public btVector3 localGetSupportingVertexWithoutMargin(final btVector3 vec0) {
  final btVector3 supVec = new btVector3();
  float maxDot = -BT_LARGE_FLOAT;
  final btVector3 vec = new btVector3(vec0);
  float lenSqr = vec.lengthSquared();
  if (lenSqr < (SIMD_EPSILON * SIMD_EPSILON)) {
   vec.set(1f, 0f, 0f);
  } else {
   float rlen = (1.f) / btSqrt(lenSqr);
   vec.scale(rlen);
  }
  int numSpheres = m_localPositionArray.size();
  btVector3[] temp = new btVector3[btMin(numSpheres, 128)];
  init(temp);
  final btVector3 a = new btVector3();
  final btVector3 b = new btVector3();
  final btVector3 c = new btVector3();
  float[] newDot = new float[1];
  int ipos = 0;
  for (int k = 0; k < numSpheres; k += 128) {
   int inner_count = btMin(numSpheres - k, 128);
   for (int i = 0; i < inner_count; i++) {
    final btVector3 pos = m_localPositionArray.get(ipos);
    float rad = m_radiArray.get(ipos);
    a.set(pos).mul(m_localScaling);
    b.set(vec).mul(m_localScaling).scale(rad);
    c.set(vec).scale(getMargin());
    temp[i].set(a).add(b).sub(c);
    //temp[i] = (*pos)*m_localScaling +vec*m_localScaling*(*rad) - vec * getMargin();
    ++ipos;
   }
   int i = vec.maxDot(temp, inner_count, newDot);
   if (newDot[0] > maxDot) {
    maxDot = newDot[0];
    supVec.set(temp[i]);
   }
  }
  return supVec;
 }

 @Override
 public void batchedUnitVectorGetSupportingVertexWithoutMargin(
  btVector3[] vectors,
  btVector3[] supportVerticesOut, int numVectors) {
  int numSpheres = m_localPositionArray.size();
  btVector3[] temp = new btVector3[btMin(numSpheres, 128)];
  init(temp);
  final btVector3 a = new btVector3();
  final btVector3 b = new btVector3();
  final btVector3 c = new btVector3();
  for (int j = 0; j < numVectors; j++) {
   float maxDot = -BT_LARGE_FLOAT;
   final btVector3 vec = vectors[j];
   float[] newDot = new float[1];
   int ipos = 0;
   for (int k = 0; k < numSpheres; k += 128) {
    int inner_count = btMin(numSpheres - k, 128);
    for (int i = 0; i < inner_count; i++) {
     final btVector3 pos = m_localPositionArray.get(ipos);
     float rad = m_radiArray.get(ipos);
     a.set(pos).mul(m_localScaling);
     b.set(vec).mul(m_localScaling).scale(rad);
     c.set(vec).scale(getMargin());
     temp[i].set(a).add(b).sub(c);
     //temp[i] = (*pos)*m_localScaling +vec*m_localScaling*(*rad) - vec * getMargin();
     ipos++;
    }
    int i = vec.maxDot(temp, inner_count, newDot);
    if (newDot[0] > maxDot) {
     maxDot = newDot[0];
     if (supportVerticesOut[j] == null) {
      supportVerticesOut[j] = new btVector3();
     }
     supportVerticesOut[j].set(temp[i]);
    }
   }
  }
 }

 public int getSphereCount() {
  return m_localPositionArray.size();
 }

 public btVector3 getSpherePosition(int index) {
  return new btVector3(m_localPositionArray.get(index));
 }

 public float getSphereRadius(int index) {
  return m_radiArray.get(index);
 }

 @Override
 public String getName() {
  return "MultiSphere";
 }

 @Override
 public int hashCode() {
  int hash = 3;
  hash += super.hashCode();
  hash = 67 * hash + Objects.hashCode(this.m_localPositionArray);
  hash = 67 * hash + Objects.hashCode(this.m_radiArray);
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
  final btMultiSphereShape other = (btMultiSphereShape) obj;
  if (!Objects.equals(this.m_localPositionArray, other.m_localPositionArray)) {
   return false;
  }
  if (!Objects.equals(this.m_radiArray, other.m_radiArray)) {
   return false;
  }
  return super.equals(obj);
 }

}
