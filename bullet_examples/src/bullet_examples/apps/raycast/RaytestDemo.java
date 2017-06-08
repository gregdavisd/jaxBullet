/*
  * Copyright (c) 2017  
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
package bullet_examples.apps.raycast;

import Bullet.Collision.AllHitsRayResultCallback;
import Bullet.Collision.ClosestRayResultCallback;
import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btBvhTriangleMeshShape;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btConvexHullShape;
import Bullet.Collision.Shape.btCylinderShape;
import Bullet.Collision.Shape.btSphereShape;
import static Bullet.Collision.btCollisionObject.CF_ANISOTROPIC_ROLLING_FRICTION;
import Bullet.Collision.btTriangleRaycastCallback;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.CollisionObjects.btRigidBodyConstructionInfo;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.btFabs;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import Bullet.Collision.Shape.btTriangleMesh;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class RaytestDemo extends DiscreteDemoContainer {

 @Override
 public void initPhysics() {
  setUpAxis(1);
  world().setGravity(new btVector3(0, -10, 0));
  ///create a few basic rigid bodies
  btCollisionShape groundShape = new btBoxShape(new btVector3((50.f), (50.f), (50.f)));
  final btTransform groundTransform = new btTransform();
  groundTransform.setIdentity();
  groundTransform.setOrigin(new btVector3(0, -50, 0));
  //We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
  {
   float mass = (0.f);
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   boolean isDynamic = (mass != 0.f);
   final btVector3 localInertia = new btVector3();
   if (isDynamic) {
    groundShape.calculateLocalInertia(mass, localInertia);
   }
   //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
   btDefaultMotionState myMotionState = new btDefaultMotionState(groundTransform);
   btRigidBodyConstructionInfo rbInfo = new btRigidBodyConstructionInfo(mass, myMotionState,
    groundShape, localInertia);
   btRigidBody body = new btRigidBody(rbInfo);
   body.setFriction(1);
   //add the body to the dynamics world
   world().addRigidBody(body);
  }
  {
   final float[] convexPoints = {-1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 2, 0, 0};
   btVector3 quad[] = {
    new btVector3(0, 1, -1),
    new btVector3(0, 1, 1),
    new btVector3(0, -1, 1),
    new btVector3(0, -1, -1)};
   btTriangleMesh mesh = new btTriangleMesh();
   mesh.addTriangle(quad[0], quad[1], quad[2], true);
   mesh.addTriangle(quad[0], quad[2], quad[3], true);
   btBvhTriangleMeshShape trimesh = new btBvhTriangleMeshShape(mesh, true, true);
   //btGImpactMeshShape * trimesh = new btGImpactMeshShape(mesh);
   //trimesh.updateBound();
   final int NUM_SHAPES = 6;
   btCollisionShape[] colShapes = {
    trimesh,
    new btConvexHullShape(convexPoints),
    new btSphereShape(1),
    new btCapsuleShape(0.2f, 1),
    new btCylinderShape(new btVector3(0.2f, 1f, 0.2f)),
    new btBoxShape(new btVector3(1, 1, 1))
   };
   for (int i = 0; i < 6; i++) {
    //create a few dynamic rigidbodies
    // Re-using the same collision is better for memory usage and performance
    /// Create Dynamic Objects
    final btTransform startTransform = new btTransform();
    startTransform.setIdentity();
    startTransform.setOrigin(new btVector3((i - 3) * 5, 1, 0));
    float mass = (1.f);
    if (i == 0) {
     mass = 0.f;
    }
    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    boolean isDynamic = (mass != 0.f);
    final btVector3 localInertia = new btVector3();
    btCollisionShape colShape = colShapes[i % NUM_SHAPES];
    if (isDynamic) {
     colShape.calculateLocalInertia(mass, localInertia);
    }
    btRigidBodyConstructionInfo rbInfo = new btRigidBodyConstructionInfo(mass, null, colShape,
     localInertia);
    rbInfo.m_startWorldTransform.set(startTransform);
    btRigidBody body = new btRigidBody(rbInfo);
    body.setRollingFriction(0.03f);
    body.setSpinningFriction(0.03f);
    body.setFriction(1);
    body.setAnisotropicFriction(colShape.getAnisotropicRollingFrictionDirection(),
     CF_ANISOTROPIC_ROLLING_FRICTION);
    world().addRigidBody(body);
   }
  }
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.024533471f, 0.58455706f, 0.01768808f, 0.8107888f), new btVector3(
   -31.176811f, 2.5281765f, 7.9228487f));
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
  return "Cast rays using the btCollisionWorld::rayTest method. The example shows how to receive the hit position and normal along the ray against the first object. Also it shows how to receive all the hits along a ray.";
 }
 static float up = 0.f;
 static float dir = 1.f;
 static float angle = 0.f;

 @Override
 public boolean render_scene() {
  boolean activity = super.render_scene();
  castRays();
  return activity;
 }

 private void castRays() {
  //add some simple animation
  //if (!m_idle)
  {
   up += 0.01 * dir;
   if (btFabs(up) > 2) {
    dir *= -1.f;
   }
   final btTransform tr = world().getCollisionObjectArray().get(1).getWorldTransform();
   angle += 0.01f;
   tr.set3x3(new btQuaternion(new btVector3(0, 1, 0), angle));
   world().getCollisionObjectArray().get(1).setWorldTransform(tr);
  }
  ///step the simulation
  world().updateAabbs();
  world().computeOverlappingPairs();
  final btVector3 red = new btVector3(1, 0, 0);
  final btVector3 blue = new btVector3(0, 0, 1);
  ///all hits
  {
   final btVector3 from = new btVector3(-30, 1 + up, 0);
   final btVector3 to = new btVector3(30, 1, 0);
   world().getDebugDrawer().drawLine(from, to, new btVector3(0, 0, 0));
   AllHitsRayResultCallback allResults = new AllHitsRayResultCallback(from, to);
   allResults.m_flags |= btTriangleRaycastCallback.kF_KeepUnflippedNormal;
   //kF_UseGjkConvexRaytest flag is now enabled by default, use the faster but more approximate algorithm
   //allResults.m_flags |= btTriangleRaycastCallback.kF_UseSubSimplexConvexCastRaytest;
   allResults.m_flags |= btTriangleRaycastCallback.kF_UseSubSimplexConvexCastRaytest;
   world().rayTest(from, to, allResults);
   for (int i = 0; i < allResults.m_hitFractions.size(); i++) {
    final btVector3 p = new btVector3(from).mix(to, allResults.m_hitFractions.get(i));
    world().getDebugDrawer().drawSphere(p, 0.1f, red);
    world().getDebugDrawer().drawLine(p, new btVector3(p).add(allResults.m_hitNormalWorld.get(i)),
     red);
   }
  }
  ///first hit
  {
   final btVector3 from = new btVector3(-30f, 1.2f, 0f);
   final btVector3 to = new btVector3(30f, 1.2f, 0f);
   world().getDebugDrawer().drawLine(from, to, new btVector3(0, 0, 1));
   ClosestRayResultCallback closestResults = new ClosestRayResultCallback(from, to);
   closestResults.m_flags |= btTriangleRaycastCallback.kF_FilterBackfaces;
   world().rayTest(from, to, closestResults);
   if (closestResults.hasHit()) {
    final btVector3 p = new btVector3(from).mix(to, closestResults.m_closestHitFraction);
    world().getDebugDrawer().drawSphere(p, 0.1f, blue);
    world().getDebugDrawer()
     .drawLine(p, new btVector3(p).add(closestResults.m_hitNormalWorld), blue);
   }
  }
 }
}
