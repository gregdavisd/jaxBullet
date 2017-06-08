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
///September 2006: VehicleDemo is work in progress, this file is mostly just a placeholder
///This VehicleDemo file is very early in development, please check it later
///@todo is a basic engine model:
///A function that maps user input (throttle) into torque/force applied on the wheels
///with gears etc.
package bullet_examples.apps.vehicle;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.INVALID_SHAPE_PROXYTYPE;
import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.Shape.btCylinderShapeX;
import Bullet.Collision.Shape.btTriangleIndexVertexArray;
import static Bullet.Collision.btCollisionObject.DISABLE_DEACTIVATION;
import static Bullet.Dynamics.Constraint.btConstraintSolverType.BT_MLCP_SOLVER;
import Bullet.Dynamics.Constraint.btHingeConstraint;
import Bullet.Dynamics.Constraint.btSliderConstraint;
import Bullet.Dynamics.ConstraintSolver.btSequentialImpulseConstraintSolver;
import Bullet.Dynamics.btDiscreteDynamicsWorld;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.CollisionObjects.btRigidBodyConstructionInfo;
import Bullet.Dynamics.vehicle.btDefaultVehicleRaycaster;
import Bullet.Dynamics.vehicle.btRaycastVehicle;
import Bullet.Dynamics.vehicle.btVehicleRaycaster;
import Bullet.Dynamics.vehicle.btVehicleTuning;
import Bullet.Dynamics.vehicle.btWheelInfo;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.M_PI;
import static Bullet.LinearMath.btScalar.M_PI_2;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import Bullet.stubs.btDantzigSolver;
import Bullet.stubs.btMLCPSolver;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;
import org.lwjgl.input.Keyboard;

/**
 *
 * @author Gregery Barton
 */
public class ForkLiftDemo extends DiscreteDemoContainer {

 private static final int CUBE_HALF_EXTENTS = 1;
 static int totalFailures = 0;
 int rightIndex = 0;
 int upIndex = 1;
 int forwardIndex = 2;
 final btVector3 wheelDirectionCS0 = new btVector3(0, -1, 0);
 final btVector3 wheelAxleCS = new btVector3(-1, 0, 0);
 float suspensionRestLength = 0.6f;
 btRigidBody m_carChassis;
 int[] m_wheelInstances = new int[4];
//----------------------------
 btRigidBody m_liftBody;
 final btVector3 m_liftStartPos = new btVector3();
 btHingeConstraint m_liftHinge;
 btRigidBody m_forkBody;
 final btVector3 m_forkStartPos = new btVector3();
 btSliderConstraint m_forkSlider;
 btRigidBody m_loadBody;
 final btVector3 m_loadStartPos = new btVector3();
 boolean m_useDefaultCamera;
//----------------------------
 btTriangleIndexVertexArray m_indexVertexArrays;
 btVector3[] m_vertices;
 btVehicleTuning m_tuning = new btVehicleTuning();
 btVehicleRaycaster m_vehicleRayCaster;
 btRaycastVehicle m_vehicle;
 btCollisionShape m_wheelShape;
 float m_cameraHeight;
 float m_minCameraDistance;
 float m_maxCameraDistance;
 boolean isShiftPressed;
 float maxMotorImpulse = 4000.f;
//the sequential impulse solver has difficulties dealing with large mass ratios (differences), between loadMass and the fork parts
 float loadMass = 350.f / 8f;//
//float loadMass = 10.f;//this should work fine for the SI solver
 boolean useMCLPSolver = false;
///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
 float gEngineForce = 0.f;
 float defaultBreakingForce = 10.f;
 float gBreakingForce = 100.f;
 float maxEngineForce = 1000.f;//this should be engine/velocity dependent
 float maxBreakingForce = 100.f;
 float gVehicleSteering = 0.f;
 float steeringIncrement = 0.04f;
 float steeringClamp = 0.3f;
 float wheelRadius = 0.5f;
 float wheelWidth = 0.4f;
 float wheelFriction = 1000;//BT_LARGE_FLOAT;
 float suspensionStiffness = 20.f;
 float suspensionDamping = 2.3f;
 float suspensionCompression = 4.4f;
 float rollInfluence = 0.1f;//1.0f;

 public ForkLiftDemo() {
  m_cameraHeight = 4.f;
  m_minCameraDistance = 3.f;
  m_maxCameraDistance = 10.f;
  m_useDefaultCamera = false;
 }

 btRigidBody localCreateRigidBody(float mass, final btTransform startTransform,
  btCollisionShape shape) {
  assert ((null == shape || shape.getShapeType() != INVALID_SHAPE_PROXYTYPE));
  //rigidbody is dynamic if and only if mass is non zero, otherwise static
  boolean isDynamic = (mass != 0.f);
  final btVector3 localInertia = new btVector3();
  if (isDynamic) {
   shape.calculateLocalInertia(mass, localInertia);
  }
  //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
  btDefaultMotionState myMotionState = new btDefaultMotionState(startTransform);
  btRigidBodyConstructionInfo cInfo = new btRigidBodyConstructionInfo(mass, myMotionState, shape,
   localInertia);
  btRigidBody body = new btRigidBody(cInfo);
  //body.setContactProcessingThreshold(m_defaultContactProcessingThreshold);
  world().addRigidBody(body);
  return body;
 }

 void lockLiftHinge() {
  float hingeAngle = m_liftHinge.getHingeAngle();
  float lowLim = m_liftHinge.getLowerLimit();
  float hiLim = m_liftHinge.getUpperLimit();
  m_liftHinge.enableAngularMotor(false, 0, 0);
  if (hingeAngle < lowLim) {
   m_liftHinge.setLimit(lowLim, lowLim);
  } else if (hingeAngle > hiLim) {
   m_liftHinge.setLimit(hiLim, hiLim);
  } else {
   m_liftHinge.setLimit(hingeAngle, hingeAngle);
  }
 } // ForkLiftDemo.lockLiftHinge()

 void lockForkSlider() {
  float linDepth = m_forkSlider.getLinearPos();
  float lowLim = m_forkSlider.getLowerLinLimit();
  float hiLim = m_forkSlider.getUpperLinLimit();
  m_forkSlider.setPoweredLinMotor(false);
  if (linDepth <= lowLim) {
   m_forkSlider.setLowerLinLimit(lowLim);
   m_forkSlider.setUpperLinLimit(lowLim);
  } else if (linDepth > hiLim) {
   m_forkSlider.setLowerLinLimit(hiLim);
   m_forkSlider.setUpperLinLimit(hiLim);
  } else {
   m_forkSlider.setLowerLinLimit(linDepth);
   m_forkSlider.setUpperLinLimit(linDepth);
  }
 } // ForkLiftDemo.lockForkSlider()

 @Override
 public void initPhysics() {
  int upAxis = 1;
  setUpAxis(upAxis);
  final btVector3 groundExtents = new btVector3(50, 50, 50);
  groundExtents.setElement(upAxis, 3);
  btCollisionShape groundShape = new btBoxShape(groundExtents);
  if (useMCLPSolver) {
   world().getSolverInfo().m_minimumSolverBatchSize = 1;//for direct solver it is better to have a small A matrix
  } else {
   world().getSolverInfo().m_minimumSolverBatchSize = 128;//for direct solver, it is better to solve multiple objects together, small batches have high overhead
  }
  world().getSolverInfo().m_globalCfm = 0.00001f;
  //world().setGravity(btVector3(0,0,0));
  final btTransform tr = new btTransform();
  tr.setIdentity();
  tr.setOrigin(new btVector3(0, -3, 0));
//either use heightfield or triangle mesh
//create ground object
  localCreateRigidBody(0, tr, groundShape);
  btCollisionShape chassisShape = new btBoxShape(new btVector3(1.f, 0.5f, 2.f));
  btCompoundShape compound = new btCompoundShape();
  final btTransform localTrans = new btTransform();
  localTrans.setIdentity();
//localTrans effectively shifts the center of mass with respect to the chassis
  localTrans.setOrigin(new btVector3(0, 1, 0));
  compound.addChildShape(localTrans, chassisShape);
  {
   btCollisionShape suppShape = new btBoxShape(new btVector3(0.5f, 0.1f, 0.5f));
   final btTransform suppLocalTrans = new btTransform();
   suppLocalTrans.setIdentity();
   //localTrans effectively shifts the center of mass with respect to the chassis
   suppLocalTrans.setOrigin(new btVector3(0f, 1.0f, 2.5f));
   compound.addChildShape(suppLocalTrans, suppShape);
  }
  tr.setOrigin(new btVector3(0, 0.f, 0));
  m_carChassis = localCreateRigidBody(800, tr, compound);
  m_wheelShape = new btCylinderShapeX(new btVector3(wheelWidth, wheelRadius, wheelRadius));
  {
   btCollisionShape liftShape = new btBoxShape(new btVector3(0.5f, 2.0f, 0.05f));
   final btTransform liftTrans = new btTransform();
   m_liftStartPos.set(0.0f, 2.5f, 3.05f);
   liftTrans.setIdentity();
   liftTrans.setOrigin(m_liftStartPos);
   m_liftBody = localCreateRigidBody(10, liftTrans, liftShape);
   final btTransform localA = new btTransform();
   final btTransform localB = new btTransform();
   localA.setIdentity();
   localB.setIdentity();
   localA.setBasis(new btMatrix3x3().setEulerZYX(0, M_PI_2, 0));
   localA.setOrigin(new btVector3(0.0f, 1.0f, 3.05f));
   localB.setBasis(new btMatrix3x3().setEulerZYX(0f, M_PI_2, 0f));
   localB.setOrigin(new btVector3(0.0f, -1.5f, -0.05f));
   m_liftHinge = new btHingeConstraint(m_carChassis, m_liftBody, localA, localB);
   m_liftHinge.setLimit(0.0f, 0.0f);
   world().addConstraint(m_liftHinge, true);
   btCollisionShape forkShapeA = new btBoxShape(new btVector3(1.0f, 0.1f, 0.1f));
   btCompoundShape forkCompound = new btCompoundShape();
   final btTransform forkLocalTrans = new btTransform();
   forkLocalTrans.setIdentity();
   forkCompound.addChildShape(forkLocalTrans, forkShapeA);
   btCollisionShape forkShapeB = new btBoxShape(new btVector3(0.1f, 0.02f, 0.6f));
   forkLocalTrans.setIdentity();
   forkLocalTrans.setOrigin(new btVector3(-0.9f, -0.08f, 0.7f));
   forkCompound.addChildShape(forkLocalTrans, forkShapeB);
   btCollisionShape forkShapeC = new btBoxShape(new btVector3(0.1f, 0.02f, 0.6f));
   forkLocalTrans.setIdentity();
   forkLocalTrans.setOrigin(new btVector3(0.9f, -0.08f, 0.7f));
   forkCompound.addChildShape(forkLocalTrans, forkShapeC);
   final btTransform forkTrans = new btTransform();
   m_forkStartPos.set(0.0f, 0.6f, 3.2f);
   forkTrans.setIdentity();
   forkTrans.setOrigin(m_forkStartPos);
   m_forkBody = localCreateRigidBody(5, forkTrans, forkCompound);
   localA.setIdentity();
   localB.setIdentity();
   localA.setBasis(new btMatrix3x3().setEulerZYX(0, 0, M_PI_2));
   localA.setOrigin(new btVector3(0.0f, -1.9f, 0.05f));
   localB.setBasis(new btMatrix3x3().setEulerZYX(0, 0, M_PI_2));
   localB.setOrigin(new btVector3(0.0f, 0.0f, -0.1f));
   m_forkSlider = new btSliderConstraint(m_liftBody, m_forkBody, localA, localB, true);
   m_forkSlider.setLowerLinLimit(0.1f);
   m_forkSlider.setUpperLinLimit(0.1f);
   m_forkSlider.setLowerAngLimit(0.0f);
   m_forkSlider.setUpperAngLimit(0.0f);
   world().addConstraint(m_forkSlider, true);
   btCompoundShape loadCompound = new btCompoundShape();
   btCollisionShape loadShapeA = new btBoxShape(new btVector3(2.0f, 0.5f, 0.5f));
   final btTransform loadTrans = new btTransform();
   loadTrans.setIdentity();
   loadCompound.addChildShape(loadTrans, loadShapeA);
   btCollisionShape loadShapeB = new btBoxShape(new btVector3(0.1f, 1.0f, 1.0f));
   loadTrans.setIdentity();
   loadTrans.setOrigin(new btVector3(2.1f, 0.0f, 0.0f));
   loadCompound.addChildShape(loadTrans, loadShapeB);
   btCollisionShape loadShapeC = new btBoxShape(new btVector3(0.1f, 1.0f, 1.0f));
   loadTrans.setIdentity();
   loadTrans.setOrigin(new btVector3(-2.1f, 0.0f, 0.0f));
   loadCompound.addChildShape(loadTrans, loadShapeC);
   loadTrans.setIdentity();
   m_loadStartPos.set(0.0f, 3.5f, 7.0f);
   loadTrans.setOrigin(m_loadStartPos);
   m_loadBody = localCreateRigidBody(loadMass, loadTrans, loadCompound);
  }
/// create vehicle
  {
   m_vehicleRayCaster = new btDefaultVehicleRaycaster(world());
   m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);
   ///never deactivate the vehicle
   m_carChassis.setActivationState(DISABLE_DEACTIVATION);
   world().addAction(m_vehicle);
   float connectionHeight = 1.2f;
   boolean isFrontWheel = true;
   //choose coordinate system
   m_vehicle.setCoordinateSystem(rightIndex, upIndex, forwardIndex);
   final btVector3 connectionPointCS0 = new btVector3(CUBE_HALF_EXTENTS - (0.3f * wheelWidth),
    connectionHeight, 2f * CUBE_HALF_EXTENTS - wheelRadius);
   m_vehicle.addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength,
    wheelRadius, m_tuning, isFrontWheel);
   connectionPointCS0.set(-CUBE_HALF_EXTENTS + (0.3f * wheelWidth), connectionHeight, 2f *
    CUBE_HALF_EXTENTS - wheelRadius);
   m_vehicle.addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength,
    wheelRadius, m_tuning, isFrontWheel);
   connectionPointCS0.set(-CUBE_HALF_EXTENTS + (0.3f * wheelWidth), connectionHeight, -2f *
    CUBE_HALF_EXTENTS + wheelRadius);
   isFrontWheel = false;
   m_vehicle.addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength,
    wheelRadius, m_tuning, isFrontWheel);
   connectionPointCS0.set(CUBE_HALF_EXTENTS - (0.3f * wheelWidth), connectionHeight, -2f *
    CUBE_HALF_EXTENTS + wheelRadius);
   m_vehicle.addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength,
    wheelRadius, m_tuning, isFrontWheel);
   for (int i = 0; i < m_vehicle.getNumWheels(); i++) {
    btWheelInfo wheel = m_vehicle.getWheelInfo(i);
    wheel.m_suspensionStiffness = suspensionStiffness;
    wheel.m_wheelsDampingRelaxation = suspensionDamping;
    wheel.m_wheelsDampingCompression = suspensionCompression;
    wheel.m_frictionSlip = wheelFriction;
    wheel.m_rollInfluence = rollInfluence;
   }
  }
  resetForklift();
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.16222472f, -0.8316988f, -0.31638032f, 0.4264551f), new btVector3(
   33.699688f, 37.90326f, -21.042444f));
 }

 @Override
 protected int getDebugMode() {
  return 0;
 }

 @Override
 protected JPanel getParams() {
  return null;
 }

 @Override
 public String get_description() {
  return "Simulate a fork lift vehicle with a working fork lift that can be moved using the cursor keys. The wheels collision is simplified using ray tests." +
   "Use arrow keys to drive the forklift, hold shift and arrows to drive the fork. ";
 }

 void resetForklift() {
  gVehicleSteering = 0.f;
  gBreakingForce = defaultBreakingForce;
  gEngineForce = 0.f;
  m_carChassis.setCenterOfMassTransform(btTransform.getIdentity());
  m_carChassis.setLinearVelocity(new btVector3());
  m_carChassis.setAngularVelocity(new btVector3());
  world().getBroadphase().getOverlappingPairCache().cleanProxyFromPairs(m_carChassis
   .getBroadphaseHandle(), world().getDispatcher());
  if (m_vehicle != null) {
   m_vehicle.resetSuspension();
   for (int i = 0; i < m_vehicle.getNumWheels(); i++) {
    //synchronize the wheels with the (interpolated) chassis worldtransform
    m_vehicle.updateWheelTransform(i, true);
   }
  }
  final btTransform liftTrans = new btTransform();
  liftTrans.setIdentity();
  liftTrans.setOrigin(m_liftStartPos);
  m_liftBody.activate();
  m_liftBody.setCenterOfMassTransform(liftTrans);
  m_liftBody.setLinearVelocity(new btVector3());
  m_liftBody.setAngularVelocity(new btVector3());
  final btTransform forkTrans = new btTransform();
  forkTrans.setIdentity();
  forkTrans.setOrigin(m_forkStartPos);
  m_forkBody.activate();
  m_forkBody.setCenterOfMassTransform(forkTrans);
  m_forkBody.setLinearVelocity(new btVector3());
  m_forkBody.setAngularVelocity(new btVector3());
  m_liftHinge.setLimit(0.0f, 0.0f);
  m_liftHinge.enableAngularMotor(false, 0, 0);
  m_forkSlider.setLowerLinLimit(0.1f);
  m_forkSlider.setUpperLinLimit(0.1f);
  m_forkSlider.setPoweredLinMotor(false);
  final btTransform loadTrans = new btTransform();
  loadTrans.setIdentity();
  loadTrans.setOrigin(m_loadStartPos);
  m_loadBody.activate();
  m_loadBody.setCenterOfMassTransform(loadTrans);
  m_loadBody.setLinearVelocity(new btVector3());
  m_loadBody.setAngularVelocity(new btVector3());
 }

 void specialKeyboard(int key, int x, int y) {
 }

 void specialKeyboardUp(int key, int x, int y) {
 }

 boolean mouseMoveCallback(float x, float y) {
  return false;
 }

 boolean mouseButtonCallback(int button, int state, float x, float y) {
  return false;
 }
 private boolean turning_left = false;
 private boolean turning_right = false;

 @Override
 public boolean keyboardCallback(int key, int state) {
  boolean handled = false;
  if (key == Keyboard.KEY_LSHIFT || key == Keyboard.KEY_RSHIFT) {
   isShiftPressed = state == 1;
  }
  if (state != 0) {
   if (isShiftPressed) {
    switch (key) {
     case Keyboard.KEY_LEFT: {
      m_liftHinge.setLimit(-M_PI / 16.0f, M_PI / 8.0f);
      m_liftHinge.enableAngularMotor(true, -0.1f, maxMotorImpulse);
      handled = true;
      break;
     }
     case Keyboard.KEY_RIGHT: {
      m_liftHinge.setLimit(-M_PI / 16.0f, M_PI / 8.0f);
      m_liftHinge.enableAngularMotor(true, 0.1f, maxMotorImpulse);
      handled = true;
      break;
     }
     case Keyboard.KEY_UP: {
      m_forkSlider.setLowerLinLimit(0.1f);
      m_forkSlider.setUpperLinLimit(3.9f);
      m_forkSlider.setPoweredLinMotor(true);
      m_forkSlider.setMaxLinMotorForce(maxMotorImpulse);
      m_forkSlider.setTargetLinMotorVelocity(1.0f);
      handled = true;
      break;
     }
     case Keyboard.KEY_DOWN: {
      m_forkSlider.setLowerLinLimit(0.1f);
      m_forkSlider.setUpperLinLimit(3.9f);
      m_forkSlider.setPoweredLinMotor(true);
      m_forkSlider.setMaxLinMotorForce(maxMotorImpulse);
      m_forkSlider.setTargetLinMotorVelocity(-1.0f);
      handled = true;
      break;
     }
    }
   } else {
    switch (key) {
     case Keyboard.KEY_LEFT: {
      turning_left = true;
      handled = true;
      break;
     }
     case Keyboard.KEY_RIGHT: {
      turning_right = true;
      handled = true;
      break;
     }
     case Keyboard.KEY_UP: {
      handled = true;
      gEngineForce = maxEngineForce;
      gBreakingForce = 0.f;
      break;
     }
     case Keyboard.KEY_DOWN: {
      handled = true;
      gEngineForce = -maxEngineForce;
      gBreakingForce = 0.f;
      break;
     }
     case Keyboard.KEY_F7: {
      handled = true;
      btDiscreteDynamicsWorld world = (btDiscreteDynamicsWorld) world();
      world.setLatencyMotionStateInterpolation(!world.getLatencyMotionStateInterpolation());
      System.out.printf("world latencyMotionStateInterpolation = %d\n", world
       .getLatencyMotionStateInterpolation() ? 1 : 0);
      break;
     }
     case Keyboard.KEY_F6: {
      handled = true;
      //switch solver (needs demo restart)
      useMCLPSolver = !useMCLPSolver;
      System.out.printf("switching to useMLCPSolver = %d\n", useMCLPSolver ? 1 : 0);
      if (useMCLPSolver) {
       btDantzigSolver mlcp = new btDantzigSolver();
       //btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
       btMLCPSolver sol = new btMLCPSolver(mlcp);
       constraint_solver = sol;
      } else {
       constraint_solver = new btSequentialImpulseConstraintSolver();
      }
      world().setConstraintSolver(constraint_solver);
      break;
     }
     case Keyboard.KEY_F5:
      handled = true;
      m_useDefaultCamera = !m_useDefaultCamera;
      break;
     default:
      break;
    }
   }
  } else {
   switch (key) {
    case Keyboard.KEY_UP: {
     lockForkSlider();
     gEngineForce = 0.f;
     gBreakingForce = defaultBreakingForce;
     handled = true;
     break;
    }
    case Keyboard.KEY_DOWN: {
     lockForkSlider();
     gEngineForce = 0.f;
     gBreakingForce = defaultBreakingForce;
     handled = true;
     break;
    }
    case Keyboard.KEY_LEFT:
    case Keyboard.KEY_RIGHT: {
     turning_left = false;
     turning_right = false;
     lockLiftHinge();
     handled = true;
     break;
    }
    default:
     break;
   }
  }
  return handled;
 }

 @Override
 public boolean render_scene() {
  if (turning_right) {
   gVehicleSteering -= steeringIncrement;
   if (gVehicleSteering < -steeringClamp) {
    gVehicleSteering = -steeringClamp;
   }
  }
  if (turning_left) {
   gVehicleSteering += steeringIncrement;
   if (gVehicleSteering > steeringClamp) {
    gVehicleSteering = steeringClamp;
   }
  }
  int wheelIndex = 2;
  m_vehicle.applyEngineForce(gEngineForce, wheelIndex);
  m_vehicle.setBrake(gBreakingForce, wheelIndex);
  wheelIndex = 3;
  m_vehicle.applyEngineForce(gEngineForce, wheelIndex);
  m_vehicle.setBrake(gBreakingForce, wheelIndex);
  wheelIndex = 0;
  m_vehicle.setSteeringValue(gVehicleSteering, wheelIndex);
  wheelIndex = 1;
  m_vehicle.setSteeringValue(gVehicleSteering, wheelIndex);
  if (world().getConstraintSolver().getSolverType() == BT_MLCP_SOLVER) {
   btMLCPSolver sol = (btMLCPSolver) world().getConstraintSolver();
   int numFallbacks = sol.getNumFallbacks();
   if (numFallbacks > 0) {
    totalFailures += numFallbacks;
    System.out.printf(
     "MLCP solver failed %d times, falling back to btSequentialImpulseSolver (SI)\n", totalFailures);
   }
   sol.setNumFallbacks(0);
  }
  boolean activity = super.render_scene();
  for (int i = 0; i < m_vehicle.getNumWheels(); i++) {
   //synchronize the wheels with the (interpolated) chassis worldtransform
   m_vehicle.updateWheelTransform(i, true);
   //draw wheels (cylinders)
   draw_shape(m_vehicle.getWheelInfo(i).m_worldTransform, m_wheelShape);
  }
  return activity;
 }
}
