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
package bullet_examples.apps.api;

import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.Shape.btCylinderShapeZ;
import Bullet.Collision.Shape.btStaticPlaneShape;
import Bullet.Dynamics.btRigidBody;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;
import static Bullet.Dynamics.btRigidBodyFlags.BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import static Bullet.LinearMath.btScalar.btSqrt;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 *
 * @author Gregery Barton
 */
public class GyroscopicDemo extends DiscreteDemoContainer {

 private static int[] gyroflags = {
  0,//none, no gyroscopic term
  BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT,
  BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD,
  BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY
 };
 private static String[] gyroNames = {
  "No Gyroscopic",
  "Explicit",
  "Implicit (World)",
  "Implicit (Body)"
 };
 private JLabel[] labels = new JLabel[4];

 @Override
 public void initPhysics() {
  {
   setUpAxis(1);
   world().setGravity(new btVector3());
   btVector3[] positions = {
    new btVector3(-10, 8, 4),
    new btVector3(-5, 8, 4),
    new btVector3(0, 8, 4),
    new btVector3(5, 8, 4),};
   for (int i = 0; i < 4; i++) {
    btCylinderShapeZ pin = new btCylinderShapeZ(new btVector3(0.1f, 0.1f, 0.2f));
    btBoxShape box = new btBoxShape(new btVector3(1f, 0.1f, 0.1f));
    box.setMargin(0.01f);
    pin.setMargin(0.01f);
    btCompoundShape compound = new btCompoundShape();
    compound.addChildShape(btTransform.getIdentity(), pin);
    final btTransform offsetBox = new btTransform(new btMatrix3x3().setIdentity(), new btVector3(0f,
     0f, 0.2f));
    compound.addChildShape(offsetBox, box);
    float[] masses = {0.3f, 0.1f};
    final btVector3 localInertia = new btVector3();
    final btTransform principal = new btTransform();
    compound.calculatePrincipalAxisTransform(masses, principal, localInertia);
    btRigidBody body = new btRigidBody(1f, null, compound, localInertia);
    final btTransform tr = new btTransform();
    tr.setIdentity();
    tr.setOrigin(positions[i]);
    body.setCenterOfMassTransform(tr);
    body.setAngularVelocity(new btVector3(0f, 0.1f, 10f));//51));
    //body.setLinearVelocity(btVector3(3, 0, 0));
    body.setFriction(btSqrt(1));
    world().addRigidBody(body);
    body.setFlags(gyroflags[i]);
    world().getSolverInfo().m_maxGyroscopicForce = 10.f;
    body.setDamping(0.0000f, 0.000f);
   }
   {
    //btCollisionShape* groundShape = new btBoxShape(btVector3(float(50.),float(50.),float(0.5)));
    btCollisionShape groundShape = new btStaticPlaneShape(new btVector3(0f, 1f, 0f), 0f);
    final btTransform groundTransform = new btTransform();
    groundTransform.setIdentity();
    groundTransform.setOrigin(new btVector3());
    btRigidBody groundBody;
    groundBody = createRigidBody(0, groundTransform, groundShape);
    groundBody.setFriction(btSqrt(2));
   }
  }
  final JFrame frame=frame();
  java.awt.EventQueue.invokeLater(new Runnable() {
   @Override
   public void run() {
    for (int i = 0; i < gyroNames.length; i++) {
     String gyroName = gyroNames[i];
     JLabel label = new JLabel(gyroName);
     label.setVisible(true);
     label.setOpaque(true);
     label.setHorizontalAlignment(JLabel.CENTER);
     label.setVerticalAlignment(JLabel.TOP);
     label.setBounds(100, 100, 100, 16);
     frame.add(label, 0);
     labels[i] = label;
    }
   }
  });
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(0.016998531f, -0.007998617f, -1.3600693E-4f, 0.9998236f),
   new btVector3(-0.8655993f, 9.381297f, 42.344086f));
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
  return "Show the Dzhanibekov effect using various settings of the gyroscopic term. " +
   "You can select the gyroscopic term computation using btRigidBody::setFlags," +
   "with arguments\n\nBT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT (using explicit " +
   "integration, which adds energy can lead to explosions)\n\nBT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD\n\n" +
   "BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY\n\n If you don't set any of these flags, " +
   "there is no gyroscopic term used.";
 }

 @Override
 public boolean render_scene() {
//render method names above objects
  final btVector3 camera_dir = camera().forward();
  for (int i = 0; i < world().getNumCollisionObjects(); i++) {
   btRigidBody body = btRigidBody.upcast(world().getCollisionObjectArray().get(i));
   if (body != null && body.getInvMass() > 0) {
    if (labels[i] != null) {
     final btTransform tr = body.getWorldTransform();
     final btVector3 pos = tr.getOrigin().add(new btVector3(0, 2, 0));
     final int label_i = i;
     if (new btVector3(pos).sub(camera().eye()).dot(camera_dir) > 0) {
      final btVector3 screen_pos = project(pos.x , pos.y , pos.z );
      screen_pos.x-=50;
      java.awt.EventQueue.invokeLater(new Runnable() {
       @Override
       public void run() {
        labels[label_i].setVisible(true);
        labels[label_i].setLocation((int) screen_pos.x, (int) screen_pos.y);
       }
      });
     } else {
      java.awt.EventQueue.invokeLater(new Runnable() {
       @Override
       public void run() {
        labels[label_i].setVisible(false);
       }
      });
     }
//      labels[i].setVisible(true);
    }
   }
  }
  //get_frame().revalidate();
  boolean activity = super.render_scene();
  return activity;
 }
}
