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
/// The btCompoundShape allows to store multiple other btCollisionShapes
/// This allows for moving concave collision objects. This is more general then the static concave btBvhTriangleMeshShape.

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.COMPOUND_SHAPE_PROXYTYPE;
import Bullet.Collision.Broadphase.btDbvt;
import Bullet.Collision.Broadphase.btDbvtAabbMm;
import Bullet.Collision.Broadphase.btDbvtNode;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import static java.util.Collections.swap;
import java.util.Objects;

/// It has an (optional) dynamic aabb tree to accelerate early rejection tests. 
/// @todo: This aabb tree can also be use to speed up ray tests on btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
/// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of btCompoundShape)
/**
 *
 * @author Gregery Barton
 */
public class btCompoundShape extends btCollisionShape implements Serializable {

 private final ArrayList<btCompoundShapeChild> m_children;
 private final btVector3 m_localAabbMin = new btVector3();
 private final btVector3 m_localAabbMax = new btVector3();
 private btDbvt m_dynamicAabbTree;
 ///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
 private int m_updateRevision;
 protected float m_collisionMargin;
 protected final btVector3 m_localScaling = new btVector3();

 public btCompoundShape(boolean enableDynamicAabbTree, int initialChildCapacity) {
  m_localAabbMin.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_localAabbMax.set((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
  m_dynamicAabbTree = null;
  m_updateRevision = 1;
  m_collisionMargin = 0f;
  m_localScaling.set(1.f, 1.f, 1.f);
  m_shapeType = COMPOUND_SHAPE_PROXYTYPE;
  if (enableDynamicAabbTree) {
   m_dynamicAabbTree = new btDbvt();
  }
  m_children = new ArrayList<>(0);
  m_children.ensureCapacity(initialChildCapacity);
 }

 public btCompoundShape(boolean enableDynamicAabbTree) {
  this(enableDynamicAabbTree, 0);
 }

 public btCompoundShape() {
  this(true, 0);
 }

 public void addChildShape(final btTransform localTransform,
  btCollisionShape shape) {
  m_updateRevision++;
  //m_childTransforms.push_back(localTransform);
  //m_childShapes.push_back(shape);
  btCompoundShapeChild child = new btCompoundShapeChild();
  child.m_node = null;
  child.m_transform.set(localTransform);
  child.m_childShape = shape;
  child.m_childShapeType = shape.getShapeType();
  child.m_childMargin = shape.getMargin();
  //extend the local aabbMin/aabbMax
  final btVector3 localAabbMin = new btVector3();
  final btVector3 localAabbMax = new btVector3();
  shape.getAabb(localTransform, localAabbMin, localAabbMax);
  if (m_localAabbMin.x > localAabbMin.x) {
   m_localAabbMin.x = localAabbMin.x;
  }
  if (m_localAabbMax.x < localAabbMax.x) {
   m_localAabbMax.x = localAabbMax.x;
  }
  if (m_localAabbMin.y > localAabbMin.y) {
   m_localAabbMin.y = localAabbMin.y;
  }
  if (m_localAabbMax.y < localAabbMax.y) {
   m_localAabbMax.y = localAabbMax.y;
  }
  if (m_localAabbMin.z > localAabbMin.z) {
   m_localAabbMin.z = localAabbMin.z;
  }
  if (m_localAabbMax.z < localAabbMax.z) {
   m_localAabbMax.z = localAabbMax.z;
  }
  if (m_dynamicAabbTree != null) {
   btDbvtAabbMm bounds = btDbvtAabbMm.fromMM(localAabbMin, localAabbMax);
   int index = m_children.size();
   child.m_node = m_dynamicAabbTree.insert(bounds, index, null);
  }
  m_children.add(child);
 }

 /// Remove all children shapes that contain the specified shape
 public void removeChildShape(btCollisionShape shape) {
  m_updateRevision++;
  // Find the children containing the shape specified, and remove those children.
  //note: there might be multiple children using the same shape!
  for (int i = m_children.size() - 1; i >= 0; i--) {
   if (m_children.get(i).m_childShape == shape) {
    removeChildShapeByIndex(i);
   }
  }
  recalculateLocalAabb();
 }

 public void removeChildShapeByIndex(int childShapeIndex) {
  m_updateRevision++;
  assert (childShapeIndex >= 0 && childShapeIndex < m_children.size());
  if (m_dynamicAabbTree != null) {
   m_dynamicAabbTree.remove(m_children.get(childShapeIndex).m_node);
  }
  swap(m_children, childShapeIndex, m_children.size() - 1);
  if (m_dynamicAabbTree != null) {
   btDbvtNode node = m_children.get(childShapeIndex).m_node;
   if (node != null) {
    node.dataAsInt(childShapeIndex);
   }
  }
  m_children.remove(m_children.size() - 1);
 }

 public int getNumChildShapes() {
  return (m_children.size());
 }

 public btCollisionShape getChildShape(int index) {
  return m_children.get(index).m_childShape;
 }

 public btTransform getChildTransform(int index) {
  return new btTransform(m_children.get(index).m_transform);
 }

 ///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
 public void updateChildTransform(int childIndex,
  final btTransform newChildTransform,
  boolean shouldRecalculateLocalAabb
 ) {
  m_children.get(childIndex).m_transform.set(newChildTransform);
  if (m_dynamicAabbTree != null) {
   ///update the dynamic aabb tree
   final btVector3 localAabbMin = new btVector3();
   final btVector3 localAabbMax = new btVector3();
   m_children.get(childIndex).m_childShape.getAabb(newChildTransform,
    localAabbMin, localAabbMax);
   btDbvtAabbMm bounds = btDbvtAabbMm.fromMM(localAabbMin, localAabbMax);
   //int index = m_children.size()-1;
   m_dynamicAabbTree.update(m_children.get(childIndex).m_node, bounds);
  }
  if (shouldRecalculateLocalAabb) {
   recalculateLocalAabb();
  }
 }

 public void updateChildTransform(int childIndex,
  final btTransform newChildTransform) {
  updateChildTransform(childIndex, newChildTransform, true);
 }

 public ArrayList<btCompoundShapeChild> getChildList() {
  return m_children;
 }

 ///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
 @Override
 public void getAabb(final btTransform trans, final btVector3 aabbMin,
  final btVector3 aabbMax) {
  final btVector3 localHalfExtents = new btVector3(m_localAabbMax).sub(
   m_localAabbMin).scale(0.5f);
  final btVector3 localCenter = new btVector3(m_localAabbMax)
   .add(m_localAabbMin).scale(0.5f);
  //avoid an illegal AABB when there are no children
  if (m_children.isEmpty()) {
   localHalfExtents.set(0, 0, 0);
   localCenter.set(0, 0, 0);
  }
  localHalfExtents.add(new btVector3(getMargin(), getMargin(), getMargin()));
  final btMatrix3x3 abs_b = trans.getBasis().abs();
  final btVector3 center = trans.transform(new btVector3(localCenter));
  final btVector3 extent = localHalfExtents.dot3(
   abs_b.getRow(0, new btVector3()),
   abs_b.getRow(1, new btVector3()),
   abs_b.getRow(2, new btVector3()));
  aabbMin.set(center).sub(extent);
  aabbMax.set(center).add(extent);
 }

 /**
  * Re-calculate the local Aabb. Is called at the end of removeChildShapes. Use
  * this yourself if you modify the children or their transforms.
  */
 public void recalculateLocalAabb() {
  // Recalculate the local aabb
  // Brute force, it iterates over all the shapes left.
  m_localAabbMin.set((BT_LARGE_FLOAT), (BT_LARGE_FLOAT), (BT_LARGE_FLOAT));
  m_localAabbMax.set((-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT), (-BT_LARGE_FLOAT));
  //extend the local aabbMin/aabbMax
  for (int j = 0; j < m_children.size(); j++) {
   final btVector3 localAabbMin = new btVector3();
   final btVector3 localAabbMax = new btVector3();
   m_children.get(j).m_childShape.getAabb(m_children.get(j).m_transform,
    localAabbMin, localAabbMax);
   if (m_localAabbMin.x > localAabbMin.x) {
    m_localAabbMin.x = localAabbMin.x;
   }
   if (m_localAabbMax.x < localAabbMax.x) {
    m_localAabbMax.x = localAabbMax.x;
   }
   if (m_localAabbMin.y > localAabbMin.y) {
    m_localAabbMin.y = localAabbMin.y;
   }
   if (m_localAabbMax.y < localAabbMax.y) {
    m_localAabbMax.y = localAabbMax.y;
   }
   if (m_localAabbMin.z > localAabbMin.z) {
    m_localAabbMin.z = localAabbMin.z;
   }
   if (m_localAabbMax.z < localAabbMax.z) {
    m_localAabbMax.z = localAabbMax.z;
   }
  }
 }

 @Override
 public void setLocalScaling(final btVector3 scaling) {
  for (int i = 0; i < m_children.size(); i++) {
   final btTransform childTrans = getChildTransform(i);
   final btVector3 childScale = m_children.get(i).m_childShape.getLocalScaling();
//		childScale = childScale * (childTrans.getBasis() * scaling);
   childScale.mul(scaling).div(m_localScaling);
   m_children.get(i).m_childShape.setLocalScaling(childScale);
   childTrans
    .setOrigin(childTrans
     .getOrigin()
     .mul(scaling)
     .div(m_localScaling));
   updateChildTransform(i, childTrans, false);
  }
  m_localScaling.set(scaling);
  recalculateLocalAabb();
 }

 @Override
 public btVector3 getLocalScaling() {
  return new btVector3(m_localScaling);
 }

 @Override
 public void calculateLocalInertia(float mass, final btVector3 inertia) {
  //approximation: take the inertia from the aabb for now
  final btTransform ident = new btTransform();
  ident.setIdentity();
  final btVector3 aabbMin = new btVector3();
  final btVector3 aabbMax = new btVector3();
  getAabb(ident, aabbMin, aabbMax);
  final btVector3 halfExtents = new btVector3(aabbMax).sub(aabbMin).scale(0.5f);
  float lx = (2.f) * (halfExtents.x());
  float ly = (2.f) * (halfExtents.y());
  float lz = (2.f) * (halfExtents.z());
  inertia.x = mass / ((12.0f)) * (ly * ly + lz * lz);
  inertia.y = mass / ((12.0f)) * (lx * lx + lz * lz);
  inertia.z = mass / ((12.0f)) * (lx * lx + ly * ly);
 }

 @Override
 public void setMargin(float margin) {
  m_collisionMargin = margin;
 }

 @Override
 public float getMargin() {
  return m_collisionMargin;
 }

 @Override
 public String getName() {
  return "Compound";
 }

 public btDbvt getDynamicAabbTree() {
  return m_dynamicAabbTree;
 }

 public void createAabbTreeFromChildren() {
  if (m_dynamicAabbTree == null) {
   m_dynamicAabbTree = new btDbvt();
   for (int index = 0; index < m_children.size(); index++) {
    btCompoundShapeChild child = m_children.get(index);
    //extend the local aabbMin/aabbMax
    final btVector3 localAabbMin = new btVector3();
    final btVector3 localAabbMax = new btVector3();
    child.m_childShape.getAabb(child.m_transform, localAabbMin, localAabbMax);
    btDbvtAabbMm bounds = btDbvtAabbMm.fromMM(localAabbMin, localAabbMax);
    int index2 = index;
    child.m_node = m_dynamicAabbTree.insert(bounds, index2, null);
   }
  }
 }

 ///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
 ///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
 ///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
 ///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
 ///of the collision object by the principal transform.
 public void calculatePrincipalAxisTransform(float[] masses,
  final btTransform principal,
  final btVector3 inertia) {
  int n = m_children.size();
  float totalMass = 0;
  final btVector3 center = new btVector3();
  int k;
  for (k = 0; k < n; k++) {
   assert (masses[k] > 0);
   center.add(m_children.get(k).m_transform.getOrigin().scale(masses[k]));
   totalMass += masses[k];
  }
  assert (totalMass > 0);
  center.scale(1.0f / totalMass);
  principal.setOrigin(center);
  final btMatrix3x3 tensor = new btMatrix3x3();
  for (k = 0; k < n; k++) {
   final btVector3 i = new btVector3();
   m_children.get(k).m_childShape.calculateLocalInertia(masses[k], i);
   final btTransform t = m_children.get(k).m_transform;
   final btVector3 o = t.getOrigin().sub(center);
   //compute inertia tensor in coordinate system of compound shape
   final btMatrix3x3 j = t.getBasis().transpose();
   j.setRow(0, j.getRow(0).scale(i.x));
   j.setRow(1, j.getRow(1).scale(i.y));
   j.setRow(2, j.getRow(2).scale(i.z));
   j.set(t.getBasis().mul(j));
   //add inertia tensor
   tensor.setRow(0, tensor.getRow(0).add(j.getRow(0)));
   tensor.setRow(1, tensor.getRow(1).add(j.getRow(1)));
   tensor.setRow(2, tensor.getRow(2).add(j.getRow(2)));
   //compute inertia tensor of pointmass at o
   float o2 = o.lengthSquared();
   j.setRow(0, o2, 0, 0);
   j.setRow(1, 0, o2, 0);
   j.setRow(2, 0, 0, o2);
   j.setRow(0, j.getRow(0).add(new btVector3(o).scale(-o.x)));
   j.setRow(1, j.getRow(1).add(new btVector3(o).scale(-o.y)));
   j.setRow(2, j.getRow(2).add(new btVector3(o).scale(-o.z)));
   //add inertia tensor of pointmass
   tensor.setRow(0, tensor.getRow(0).add(j.getRow(0).scale(masses[k])));
   tensor.setRow(1, tensor.getRow(1).add(j.getRow(1).scale(masses[k])));
   tensor.setRow(2, tensor.getRow(2).add(j.getRow(2).scale(masses[k])));
  }
  tensor.diagonalize(principal.getBasis(), 0.00001f, 20);
  inertia.set(tensor.m00, tensor.m11, tensor.m22);
 }

 public int getUpdateRevision() {
  return m_updateRevision;
 }

 @Override
 public int hashCode() {
  int hash = 7;
  hash += super.hashCode();
  hash = 47 * hash + Objects.hashCode(this.m_children);
  hash = 47 * hash + Objects.hashCode(this.m_localAabbMin);
  hash = 47 * hash + Objects.hashCode(this.m_localAabbMax);
  hash = 47 * hash + Float.floatToIntBits(this.m_collisionMargin);
  hash = 47 * hash + Objects.hashCode(this.m_localScaling);
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
  final btCompoundShape other = (btCompoundShape) obj;
  if (Float.floatToIntBits(this.m_collisionMargin) != Float.floatToIntBits(
   other.m_collisionMargin)) {
   return false;
  }
  if (!Objects.equals(this.m_children, other.m_children)) {
   return false;
  }
  if (!Objects.equals(this.m_localAabbMin, other.m_localAabbMin)) {
   return false;
  }
  if (!Objects.equals(this.m_localAabbMax, other.m_localAabbMax)) {
   return false;
  }
  if (!Objects.equals(this.m_localScaling, other.m_localScaling)) {
   return false;
  }
  return super.equals(obj);
 }

}
