/*
 * Bullet Continuous Collision Detection and Physics Library
 * Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
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
package Bullet.Dynamics;

import Bullet.Collision.Broadphase.btBroadphaseInterface;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.btCollisionConfiguration;
import Bullet.Collision.btCollisionWorld;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.ConstraintSolver.btConstraintSolver;
import Bullet.LinearMath.btVector3;
import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
abstract public class btDynamicsWorld extends btCollisionWorld implements
 Serializable {

 btInternalTickCallback m_internalTickCallback;
 btInternalTickCallback m_internalPreTickCallback;
 Object m_worldUserInfo;
 final btContactSolverInfo m_solverInfo = new btContactSolverInfo();

 public btDynamicsWorld(btDispatcher dispatcher,
  btBroadphaseInterface broadphase,
  btCollisionConfiguration collisionConfiguration) {
  super(dispatcher, broadphase, collisionConfiguration);
  m_internalTickCallback = null;
  m_internalPreTickCallback = null;
  m_worldUserInfo = null;
 }

 ///stepSimulation proceeds the simulation over 'timeStep', units in preferably in seconds.
 ///By default, Bullet will subdivide the timestep in constant substeps of each 'fixedTimeStep'.
 ///in order to keep the simulation real-time, the maximum number of substeps can be clamped to 'maxSubSteps'.
 ///You can disable subdividing the timestep/substepping by passing maxSubSteps=0 as second argument to stepSimulation, but in that case you have to keep the timeStep constant.
 public abstract int stepSimulation(float timeStep, int maxSubSteps,
  float fixedTimeStep);

 public final int stepSimulation(float timeStep) {
  assert (timeStep >= 0);
  return stepSimulation(timeStep, 1);
 }

 public final int stepSimulation(float timeStep, int maxSubSteps) {
  return stepSimulation(timeStep, maxSubSteps, 1f / 60f);
 }

 //public abstract void debugDrawWorld();
 public void addConstraint(btTypedConstraint constraint,
  boolean disableCollisionsBetweenLinkedBodies) {
  //(void)constraint; (void)disableCollisionsBetweenLinkedBodies;
 }

 public void addConstraint(btTypedConstraint constraint) {
  addConstraint(constraint, false);
 }

 public void removeConstraint(btTypedConstraint constraint) {
  //	(void)constraint;
 }

 public abstract void addAction(btActionInterface action);

 public abstract void removeAction(btActionInterface action);

 //once a rigidbody is added to the dynamics world, it will get this gravity assigned
 //existing rigidbodies in the world get gravity assigned too, during this method
 public abstract void setGravity(final btVector3 gravity);

 public abstract btVector3 getGravity();

 public abstract void synchronizeMotionStates();

 public abstract void addRigidBody(btRigidBody body);

 public abstract void addRigidBody(btRigidBody body, int group, int mask);

 public abstract void removeRigidBody(btRigidBody body);

 public abstract void setConstraintSolver(btConstraintSolver solver);

 public abstract btConstraintSolver getConstraintSolver();

 public int getNumConstraints() {
  return 0;
 }

 public btTypedConstraint getConstraint(int index) {
  //(void)index;		
  return null;
 }

 public abstract int getWorldType();

 abstract void clearForces();

 /// Set the callback for when an internal tick (simulation substep) happens, optional user info
 public void setInternalTickCallback(btInternalTickCallback cb,
  Object worldUserInfo,
  boolean isPreTick) {
  if (isPreTick) {
   m_internalPreTickCallback = cb;
  } else {
   m_internalTickCallback = cb;
  }
  m_worldUserInfo = worldUserInfo;
 }

 public void setInternalTickCallback(btInternalTickCallback cb,
  Object worldUserInfo) {
  setInternalTickCallback(cb, worldUserInfo, false);
 }

 public void setInternalTickCallback(btInternalTickCallback cb) {
  setInternalTickCallback(cb, null, false);
 }

 public void setWorldUserInfo(Object worldUserInfo) {
  m_worldUserInfo = worldUserInfo;
 }

 public Object getWorldUserInfo() {
  return m_worldUserInfo;
 }

 public btContactSolverInfo getSolverInfo() {
  return m_solverInfo;
 }

 /*
  * ///obsolete, use addAction instead. public void addVehicle(btActionInterface
  * vehicle) { //(void)vehicle; } ///obsolete, use removeAction instead
  *
  * public void removeVehicle(btActionInterface vehicle) { //(void)vehicle; }
  * ///obsolete, use addAction instead.
  *
  * public void addCharacter(btActionInterface character) { //(void)character; }
  * ///obsolete, use removeAction instead
  *
  * public void removeCharacter(btActionInterface character) {
  * //(void)character; }
  */
}
