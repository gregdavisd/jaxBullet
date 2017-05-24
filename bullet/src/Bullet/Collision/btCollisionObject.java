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

import Bullet.Collision.Broadphase.btBroadphaseProxy;
import static Bullet.Collision.CollisionFlags.CF_HAS_CONTACT_STIFFNESS_DAMPING;
import static Bullet.Collision.CollisionFlags.CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR;
import static Bullet.Collision.CollisionFlags.CF_KINEMATIC_OBJECT;
import static Bullet.Collision.CollisionFlags.CF_NO_CONTACT_RESPONSE;
import static Bullet.Collision.CollisionFlags.CF_STATIC_OBJECT;
import static Bullet.Collision.CollisionObjectTypes.CO_COLLISION_OBJECT;
import Bullet.Collision.Shape.btCollisionShape;
import static Bullet.LinearMath.btScalar.BT_LARGE_FLOAT;
import static Bullet.LinearMath.btScalar.SIMD_EPSILON;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import java.util.ArrayList;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import static javax.vecmath.VecMath.is_good_matrix;

/**
 *
 * @author Gregery Barton
 */
/// btCollisionObject can be used to manage collision detection objects. 
/// btCollisionObject maintains all information that is needed for a collision detection: Shape, Transform and AABB proxy.
/// They can be added to the btCollisionWorld.
public class btCollisionObject  implements Serializable  {

//island management, m_activationState1
 public static final int ACTIVE_TAG = 1;
 public static final int ISLAND_SLEEPING = 2;
 public static final int WANTS_DEACTIVATION = 3;
 public static final int DISABLE_DEACTIVATION = 4;
 public static final int DISABLE_SIMULATION = 5;
 public static final int CF_ANISOTROPIC_FRICTION_DISABLED = 0;
 public static final int CF_ANISOTROPIC_FRICTION = 1;
 public static final int CF_ANISOTROPIC_ROLLING_FRICTION = 2;
 protected final btTransform m_worldTransform = new btTransform();
 ///m_interpolationWorldTransform is used for CCD and interpolation
 ///it can be either previous or future (predicted) transform
 protected final btTransform m_interpolationWorldTransform = new btTransform();
 //those two are experimental: just added for bullet time effect, so you can still apply impulses (directly modifying velocities) 
 //without destroying the continuous interpolated motion (which uses this interpolation velocities)
 protected final btVector3 m_interpolationLinearVelocity = new btVector3();
 protected final btVector3 m_interpolationAngularVelocity = new btVector3();
 protected final btVector3 m_anisotropicFriction = new btVector3();
 protected int m_hasAnisotropicFriction;
 protected float m_contactProcessingThreshold;
 protected btBroadphaseProxy m_broadphaseHandle;
 protected btCollisionShape m_collisionShape;
 ///m_extensionPointer is used by some internal low-level Bullet extensions.
 protected Object m_extensionPointer;
 ///m_rootCollisionShape is temporarily used to store the original collision shape
 ///The m_collisionShape might be temporarily replaced by a child collision shape during collision detection purposes
 ///If it is null, the m_collisionShape is not temporarily replaced.
 protected btCollisionShape m_rootCollisionShape;
 protected int m_collisionFlags;
 protected int m_islandTag1;
 //protected int m_companionId;
 protected Object m_companion;
 protected int m_worldArrayIndex;  // index of object in world's collisionObjects array
 protected int m_activationState1;
 protected float m_deactivationTime;
 protected float m_friction;
 protected float m_restitution;
 protected float m_rollingFriction;//torsional friction orthogonal to contact normal (useful to stop spheres rolling forever)
 protected float m_spinningFriction; // torsional friction around the contact normal (useful for grasping)
 protected float m_contactDamping;
 protected float m_contactStiffness;
 ///m_internalType is reserved to distinguish Bullet's btCollisionObject, btRigidBody, btSoftBody, btGhostObject etc.
 ///do not assign your own m_internalType unless you write a new dynamics object class.
 protected int m_internalType;
 ///users can point to their objects, m_userPointer is not used by Bullet, see setUserPointer/getUserPointer
 protected Object m_userObjectPointer;
 protected int m_userIndex2;
 protected int m_userIndex;
 ///time of impact calculation
 protected float m_hitFraction;
 ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm.
 protected float m_ccdSweptSphereRadius;
 /// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
 protected float m_ccdMotionThreshold;
 /// If some object should have elaborate collision filtering by sub-classes
 protected boolean m_checkCollideWith;
 protected final ArrayList<btCollisionObject> m_objectsWithoutCollisionCheck = new ArrayList<>(0);
 ///internal update revision number. It will be increased when the object changes. This allows some subsystems to perform lazy evaluation.
 protected final btVector3 m_customDebugColorRGB = new btVector3();
 //public String debug_name = "";

 public btCollisionObject() {
  m_anisotropicFriction.set(1.f, 1.f, 1.f);
  m_hasAnisotropicFriction = 0;
  m_contactProcessingThreshold = BT_LARGE_FLOAT;
  m_broadphaseHandle = null;
  m_collisionShape = null;
  m_extensionPointer = null;
  m_rootCollisionShape = null;
  m_collisionFlags = CF_STATIC_OBJECT;
  m_islandTag1 = -1;
  //m_companionId = -1;
  m_worldArrayIndex = -1;
  m_activationState1 = 1;
  m_deactivationTime = 0;
  m_friction = (0.5f);
  m_restitution = (0);
  m_rollingFriction = 0.0f;
  m_spinningFriction = (0.f);
  m_contactDamping = (.1f);
  m_contactStiffness = (1e4f);
  m_internalType = (CO_COLLISION_OBJECT);
  m_userObjectPointer = null;
  m_userIndex2 = -1;
  m_userIndex = -1;
  m_hitFraction = 1.f;
  m_ccdSweptSphereRadius = 0;
  m_ccdMotionThreshold = 0;
  m_checkCollideWith = false;
  m_worldTransform.setIdentity();
  assert (is_good_matrix(m_worldTransform));
 }

 public boolean mergesSimulationIslands() {
  ///static objects, kinematic and object without contact response don't merge islands
  return ((m_collisionFlags & (CF_STATIC_OBJECT | CF_KINEMATIC_OBJECT | CF_NO_CONTACT_RESPONSE)) ==
   0);
 }

 public btVector3 getAnisotropicFriction() {
  return new btVector3(m_anisotropicFriction);
 }

 public void setAnisotropicFriction(final btVector3 anisotropicFriction, int frictionMode) {
  m_anisotropicFriction.set(anisotropicFriction);
  boolean isUnity = (anisotropicFriction.x != 1.f) || (anisotropicFriction.y != 1.f) ||
   (anisotropicFriction.z != 1.f);
  m_hasAnisotropicFriction = isUnity ? frictionMode : 0;
 }

 public void setAnisotropicFriction(final btVector3 anisotropicFriction) {
  setAnisotropicFriction(anisotropicFriction, CF_ANISOTROPIC_FRICTION);
 }

 public boolean hasAnisotropicFriction(int frictionMode) {
  return (m_hasAnisotropicFriction & frictionMode) != 0;
 }

 public boolean hasAnisotropicFriction() {
  return hasAnisotropicFriction(CF_ANISOTROPIC_FRICTION);
 }

 ///the constraint solver can discard solving contacts, if the distance is above this threshold. 0 by default.
 ///Note that using contacts with positive distance can improve stability. It increases, however, the chance of colliding with degerate contacts, such as 'interior' triangle edges
 public void setContactProcessingThreshold(float contactProcessingThreshold) {
  m_contactProcessingThreshold = contactProcessingThreshold;
 }

 public float getContactProcessingThreshold() {
  return m_contactProcessingThreshold;
 }

 public boolean isStaticObject() {
  return (m_collisionFlags & CF_STATIC_OBJECT) != 0;
 }

 public boolean isKinematicObject() {
  return (m_collisionFlags & CF_KINEMATIC_OBJECT) != 0;
 }

 public boolean isStaticOrKinematicObject() {
  return (m_collisionFlags & (CF_KINEMATIC_OBJECT | CF_STATIC_OBJECT)) != 0;
 }

 public boolean hasContactResponse() {
  return (m_collisionFlags & CF_NO_CONTACT_RESPONSE) == 0;
 }

 public void setCollisionShape(btCollisionShape collisionShape) {
  m_collisionShape = collisionShape;
  m_rootCollisionShape = collisionShape;
 }

 public btCollisionShape getCollisionShape() {
  return m_collisionShape;
 }

 public void setIgnoreCollisionCheck(btCollisionObject co, boolean ignoreCollisionCheck
 ) {
  if (ignoreCollisionCheck) {
   //We don't check for duplicates. Is it ok to leave that up to the user of this API?
   //int index = m_objectsWithoutCollisionCheck.findLinearSearch(co);
   //if (index == m_objectsWithoutCollisionCheck.size())
   //{
   m_objectsWithoutCollisionCheck.add(co);
   //}
  } else {
   m_objectsWithoutCollisionCheck.remove(co);
  }
  m_checkCollideWith = m_objectsWithoutCollisionCheck.size() > 0;
 }

 public boolean checkCollideWithOverride(btCollisionObject co) {
  int index = m_objectsWithoutCollisionCheck.indexOf(co);
  return index < 0;
 }

 ///Avoid using this internal API call, the extension pointer is used by some Bullet extensions. 
 ///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
 public Object internalGetExtensionPointer() {
  return m_extensionPointer;
 }

 ///Avoid using this internal API call, the extension pointer is used by some Bullet extensions
 ///If you need to store your own user pointer, use 'setUserPointer/getUserPointer' instead.
 public void internalSetExtensionPointer(Object pointer) {
  m_extensionPointer = pointer;
 }

 public int getActivationState() {
  return m_activationState1;
 }

 public void setActivationState(int newState) {
  if ((m_activationState1 != DISABLE_DEACTIVATION) && (m_activationState1 != DISABLE_SIMULATION)) {
   m_activationState1 = newState;
  }
 }

 public void setDeactivationTime(float time) {
  m_deactivationTime = time;
 }

 public float getDeactivationTime() {
  return m_deactivationTime;
 }

 public void forceActivationState(int newState) {
  m_activationState1 = newState;
 }

 public void activate(boolean forceActivation) {
  if (forceActivation || !((m_collisionFlags & (CF_STATIC_OBJECT | CF_KINEMATIC_OBJECT)) != 0)) {
   setActivationState(ACTIVE_TAG);
   m_deactivationTime = 0;
  }
 }

 public void activate() {
  activate(false);
 }

 public boolean isActive() {
  return ((getActivationState() != ISLAND_SLEEPING) && (getActivationState() != DISABLE_SIMULATION));
 }

 public void setRestitution(float rest) {
  m_restitution = rest;
 }

 public float getRestitution() {
  return m_restitution;
 }

 public void setFriction(float frict) {
  m_friction = frict;
 }

 public float getFriction() {
  return m_friction;
 }

 public void setRollingFriction(float frict) {
  m_rollingFriction = frict;
 }

 public float getRollingFriction() {
  return m_rollingFriction;
 }

 public void setSpinningFriction(float frict) {
  m_spinningFriction = frict;
 }

 public float getSpinningFriction() {
  return m_spinningFriction;
 }

 public void setContactStiffnessAndDamping(float stiffness, float damping) {
  m_contactStiffness = stiffness;
  m_contactDamping = damping;
  m_collisionFlags |= CF_HAS_CONTACT_STIFFNESS_DAMPING;
  //avoid divisions by zero...
  if (m_contactStiffness < SIMD_EPSILON) {
   m_contactStiffness = SIMD_EPSILON;
  }
 }

 public float getContactStiffness() {
  return m_contactStiffness;
 }

 public float getContactDamping() {
  return m_contactDamping;
 }

 ///reserved for Bullet internal usage
 public int getInternalType() {
  return m_internalType;
 }

 public btTransform getWorldTransform() {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  return new btTransform(m_worldTransform);
 }
 public btTransform getWorldTransformPtr() {
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
  return   m_worldTransform;
 }

 public void setWorldTransform(final btTransform worldTrans) {
  m_worldTransform.set(worldTrans);
  if (DEBUG_BLOCKS) {
   assert (is_good_matrix(m_worldTransform));
  }
 }

 public btBroadphaseProxy getBroadphaseHandle() {
  return m_broadphaseHandle;
 }

 public void setBroadphaseHandle(btBroadphaseProxy handle) {
  m_broadphaseHandle = handle;
 }

 public btTransform getInterpolationWorldTransform() {
  return new btTransform(m_interpolationWorldTransform);
 }

 public void setInterpolationWorldTransform(final btTransform trans) {
  m_interpolationWorldTransform.set(trans);
 }

 public void setInterpolationLinearVelocity(final btVector3 linvel) {
  m_interpolationLinearVelocity.set(linvel);
 }

 public void setInterpolationAngularVelocity(final btVector3 angvel) {
  m_interpolationAngularVelocity.set(angvel);
 }

 public btVector3 getInterpolationLinearVelocity() {
  return new btVector3(m_interpolationLinearVelocity);
 }

 public btVector3 getInterpolationAngularVelocity() {
  return new btVector3(m_interpolationAngularVelocity);
 }

 public int getIslandTag() {
  return m_islandTag1;
 }

 public void setIslandTag(int tag) {
  m_islandTag1 = tag;
 }

// public int getCompanionId() {
//  return m_companionId;
// }
//
// public void setCompanionId(int id) {
//  m_companionId = id;
// }
 public Object getCompanion() {
  return m_companion;
 }

 public void setCompanion(Object companion) {
  m_companion = companion;
 }

 public int getWorldArrayIndex() {
  return m_worldArrayIndex;
 }

 // only should be called by CollisionWorld
 public void setWorldArrayIndex(int ix) {
  m_worldArrayIndex = ix;
 }

 public float getHitFraction() {
  return m_hitFraction;
 }

 public void setHitFraction(float hitFraction) {
  m_hitFraction = hitFraction;
 }

 public int getCollisionFlags() {
  return m_collisionFlags;
 }

 public void setCollisionFlags(int flags) {
  m_collisionFlags = flags;
 }

 ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm.
 public float getCcdSweptSphereRadius() {
  return m_ccdSweptSphereRadius;
 }

 ///Swept sphere radius (0.0 by default), see btConvexConvexAlgorithm.
 public void setCcdSweptSphereRadius(float radius) {
  m_ccdSweptSphereRadius = radius;
 }

 public float getCcdMotionThreshold() {
  return m_ccdMotionThreshold;
 }

 public float getCcdSquareMotionThreshold() {
  return m_ccdMotionThreshold * m_ccdMotionThreshold;
 }

 /// Don't do continuous collision detection if the motion (in one step) is less then m_ccdMotionThreshold
 public void setCcdMotionThreshold(float ccdMotionThreshold) {
  m_ccdMotionThreshold = ccdMotionThreshold;
 }

 ///users can point to their objects, userPointer is not used by Bullet
 public Object getUserPointer() {
  return m_userObjectPointer;
 }

 public int getUserIndex() {
  return m_userIndex;
 }

 public int getUserIndex2() {
  return m_userIndex2;
 }

 ///users can point to their objects, userPointer is not used by Bullet
 public void setUserPointer(Object userPointer) {
  m_userObjectPointer = userPointer;
 }

 ///users can point to their objects, userPointer is not used by Bullet
 public void setUserIndex(int index) {
  m_userIndex = index;
 }

 public void setUserIndex2(int index) {
  m_userIndex2 = index;
 }
 

 public void setCustomDebugColor(final btVector3 colorRGB) {
  m_customDebugColorRGB.set(colorRGB);
  m_collisionFlags |= CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR;
 }

 public void removeCustomDebugColor() {
  m_collisionFlags &= ~CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR;
 }

 public boolean getCustomDebugColor(final btVector3 colorRGB) {
  boolean hasCustomColor = (0 != (m_collisionFlags & CF_HAS_CUSTOM_DEBUG_RENDERING_COLOR));
  if (hasCustomColor) {
   colorRGB.set(m_customDebugColorRGB);
  }
  return hasCustomColor;
 }

 public boolean checkCollideWith(btCollisionObject co) {
  if (m_checkCollideWith) {
   return checkCollideWithOverride(co);
  }
  return true;
 }

  
}
