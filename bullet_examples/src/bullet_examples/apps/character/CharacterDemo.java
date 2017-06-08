/*
 * Copyright (c) 2017 Gregery Barton
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
package bullet_examples.apps.character;

import Bullet.Dynamics.CollisionObjects.btPairCachingGhostObject;
import static Bullet.Collision.Broadphase.btBroadphaseProxy.CHARACTER_FILTER;
import static Bullet.Collision.Broadphase.btBroadphaseProxy.DEFAULT_FILTER;
import static Bullet.Collision.Broadphase.btBroadphaseProxy.STATIC_FILTER;
import static Bullet.Collision.CollisionFlags.CF_CHARACTER_OBJECT;
import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Dynamics.CollisionObjects.btGhostPairCallback;
import Bullet.Dynamics.character.btKinematicCharacterController;
import Bullet.LinearMath.btClock;
import Bullet.LinearMath.btMatrix3x3;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import bullet_examples.DiscreteDemoContainer;
import bullet_examples.TriangleStripElement;
import java.util.concurrent.ThreadLocalRandom;
import javax.swing.JPanel;
import org.lwjgl.input.Keyboard;
import org.lwjgl.opengl.GL11;
import static org.lwjgl.opengl.GL11.glMultMatrix;

/**
 *
 * @author Gregery Barton
 */
public class CharacterDemo extends DiscreteDemoContainer {

 private static final TriangleStripElement HAT = new TriangleStripElement(
  new int[]{1052898036,
   1061043640, 1042927455, 1058060577, 1061901719, 1046729160, 1060708823,
   1052554694, -1103434395,
   1063096251, 1055585750, -1102210661, 1052898031, 1061043640,
   -1104556176, 1058072354, 1061865366,
   -1100361260, 1059072008, 1052554694, -1093923786, 1061073789,
   1054415282, -1090267384, -1103974362,
   1057067522, 1059361425, -1104494250, 1058859322, 1061533067, 1042328634,
   1061043640, 1053620892,
   1047706085, 1062182303, 1057549074, -1105155034, 1061043640, 1053620899,
   -1099810332, 1062190495,
   1057540370, -1103974330, 1052554694, -1086545743, -1102970492,
   1056842236, -1084694606,
   -1092960737, 0, -1086640695, -1088855245, 1049224468, -1086258814,
   -1094753070, 1052554694,
   -1087935386, -1090335994, 1055215051, -1086598280, -1096416413,
   1056002972, 1058731996,
   -1092005421, 1058387243, 1059834200, -1105155041, 1048401167,
   1061982376, -1105088189, 1061705617,
   1058678580, -1094585617, 1061043640, 1042927472, -1089423583,
   1061903255, 1046710727, -1086774508,
   1052554694, 1044300013, -1084435526, 1055919584, 1044533637,
   -1088411322, 1052554694, 1053685243,
   -1086794382, 1057297674, 1055505875, -1094585612, 1061043640,
   -1104556193, -1089411294, 1061865366,
   -1100361260, -1088411318, 1052554694, -1093923801, -1086409859,
   1054415282, -1090267384,
   -1086774504, 1052554694, -1103434427, -1084387397, 1055584726,
   -1102210661, -1103038593,
   1062693333, 1034482507, -1098894080, 1064737261, 1036382604,
   -1113596966, 1062693333, 1045164263,
   -1109741131, 1064761838, 1047933420, 1044445059, 1062693333, 1034482489,
   1048589568, 1064737261,
   1036382604, 1043508016, 1052554694, -1086545738, 1044513156, 1056842236,
   -1084694606, 1054522895,
   0, -1086640691, 1058628403, 1049225492, -1086258814, 1042328606, 0,
   -1084039417, 1046653381,
   1052173678, -1083684399, 1044445055, 1062693333, -1113001141,
   1048589568, 1064737261, -1111101044,
   1033886682, 1062693333, -1102319385, 1037742517, 1064761838,
   -1099550228, 1042328614, 1061043640,
   -1093862749, 1047861738, 1061886358, -1089517793, -1105155014,
   1061043640, -1093862756,
   -1099621910, 1061886358, -1089517793, -1113596947, 1062693333,
   -1102319392, -1109741131,
   1064761838, -1099550228, -1103038587, 1062693333, -1113001158,
   -1098894080, 1064737261,
   -1111101044, 1033886701, 1062693333, 1045164256, 1037742517, 1064761838,
   1047933420, 1042328643,
   1048067556, 1062041612, 1042690380, 1061758354, 1058589490, -1105155057,
   0, 1070136455,
   -1115639551, 1064529895, 1050351926, 1042328659, 0, 1070136455,
   1031934212, 1064541671, 1050272052,
   -1092245280, 0, 1068886853, -1101303369, 1064298976, 1049423130,
   -1092994850, 1044201918,
   1060703121, -1090804233, 1060793717, 1056212457, -1088506189, 0,
   1067000029, -1095002761,
   1063804881, 1045514659, 1055238400, 0, 1068886851, 1045983665,
   1064331745, 1049275669, 1054579432,
   1043826197, 1060751001, 1056624118, 1060850039, 1056094693, 1058977473,
   0, 1067000025, 1052257648,
   1063862226, 1045283228, 1061097555, 0, 1057875075, 1061079422,
   1058235175, 1051470168, 1052729930,
   1052554694, -1087935380, 1057147654, 1055216075, -1086598280,
   -1105155004, 0, -1084039419,
   -1100830267, 1052173678, -1083683887, 1060011609, 0, -1092813237,
   1062033307, 1045770666,
   -1089518305, 1059072012, 1052554694, 1053685228, 1060689778, 1057304842,
   1055487443, 1051117001,
   1055898515, 1058758615, 1055481299, 1058381611, 1059838296, 1062516348,
   0, 1042927446, 1064392675,
   1050511675, 1040412935, 1060708825, 1052554694, 1044299981, 1063048122,
   1055919584, 1044533637,
   -1087472035, 0, -1092813251, -1085450341, 1045768618, -1089518305,
   -1084967300, 0, -1104556202,
   -1083182624, 1049030926, -1100379693, -1084967302, 0, 1042927482,
   -1083090461, 1050510651,
   1040414983, -1086386099, 0, 1057875083, -1086388866, 1058230055,
   1051417943, 1043508049,
   1057021146, 1059394361, 1042999638, 1058843449, 1061544844, 1062516346,
   0, -1104556166, 1064300512,
   1049032974, -1100379693},
  new int[]{34, 35, 36, 35, 51, 14, 50, 13, 17, 12, 15, 29, 28, 25, 27, 26,
   21, 41, 22, 41, 43, 3, 1,
   2, 0, 20, 30, 19, 5, 6, 4, 10, 11, 35, 11, 34, 11, 32, 31, 33, 31, 37,
   38, 39, 38, 40, 44, 46, 47,
   1, 47, 0, 44, 45, 38, 45, 31, 52, 11, 52, 4, 52, 5, 45, 5, 0, 30, 30,
   22, 22, 22, 23, 21, 42, 7,
   8, 9, 48, 16, 17, 16, 15, 27, 28, 28, 13, 13, 14, 12, 10, 12, 6, 12, 19,
   18, 20, 29, 24, 25, 2,
   26, 3, 41, 41, 14, 14, 14, 35, 10, 10, 24, 24, 24, 20, 2, 2, 18, 18, 18,
   29, 12, 12, 48, 48, 48,
   49, 17, 50, 50, 21, 21, 7, 27, 9, 16, 16, 43, 43, 43, 1, 53, 46});
 private final float characterScale = 2f;
 public btPairCachingGhostObject ghostObject;
 public btKinematicCharacterController character;
 boolean forward;
 boolean backward;
 boolean left;
 boolean right;
 boolean jump;
 private btClock move_clock = new btClock();

 @Override
 public void initPhysics() {
  setUpAxis(1);
  createLargeMeshBody(new btVector3(1, 0.35f, 1));
  final btVector3 boxSize = new btVector3(1.5f, 1.5f, 1.5f);
  float boxMass = 1.0f;
  {
   int size = 4;
   int height = 4;
   float cubeSize = boxSize.x;
   float spacing = 2.0f;
   final btVector3 pos = new btVector3(0.0f, 20.0f, 0.0f);
   float offset = -size * (cubeSize * 2.0f + spacing) * 0.5f;
   int numBodies = 0;
   for (int k = 0; k < height; k++) {
    for (int j = 0; j < size; j++) {
     pos.z = offset + (float) j * (cubeSize * 2.0f + spacing);
     for (int i = 0; i < size; i++) {
      pos.x = offset + (float) i * (cubeSize * 2.0f + spacing);
      final btVector3 bpos = new btVector3(0, 25, 0).add(
       new btVector3(5.0f, 1.0f, 5.0f).mul(pos));
      int idx = ThreadLocalRandom.current().nextInt() % 9;
      final btTransform trans = new btTransform();
      trans.setIdentity();
      trans.setOrigin(bpos);
      switch (idx) {
       case 0:
       case 1:
       case 2:
       case 3:
       case 4:
       case 5:
       case 6:
       case 7:
       case 8: {
        float r = 0.5f * (idx + 1);
        btBoxShape boxShape = new btBoxShape(
         new btVector3(boxSize).scale(r));
        createRigidBody(boxMass * r, trans, boxShape);
       }
       break;
      }
      numBodies++;
     }
    }
    offset -= 0.05f * spacing * (size - 1);
    spacing *= 1.1f;
    pos.y += (cubeSize * 2.0f + spacing);
   }
  }
  float characterHeight = 1.75f * characterScale;
  float characterWidth = 1.75f * characterScale;
  btConvexShape capsule = new btCapsuleShape(characterWidth,
   characterHeight);
  ghostObject = new btPairCachingGhostObject();
  world().getPairCache().setInternalGhostPairCallback(
   new btGhostPairCallback());
  ghostObject.setCollisionShape(capsule);
  ghostObject.setCollisionFlags(CF_CHARACTER_OBJECT);
  float stepHeight = 0.35f * characterScale;
  character = new btKinematicCharacterController(ghostObject, capsule,
   stepHeight, new btVector3(0,
    1, 0));
  character.setJumpSpeed(20.0f);
  world.addCollisionObject(ghostObject, CHARACTER_FILTER, STATIC_FILTER
   | DEFAULT_FILTER);
  world().addAction(character);
 }

 @Override
 public void resetCamera() {
  camera().set(new btQuaternion(-0.0031397976f, 0.94800323f, 0.31810763f,
   -0.009356874f), new btVector3(4.961824f, 132.38693f, -179.54715f));
 }

 @Override
 public boolean keyboardCallback(int key, int state) {
  if (state != 0) {
   switch (key) {
    case Keyboard.KEY_UP:
     forward = true;
     break;
    case Keyboard.KEY_DOWN:
     backward = true;
     break;
    case Keyboard.KEY_LEFT:
     left = true;
     break;
    case Keyboard.KEY_RIGHT:
     right = true;
     break;
    case Keyboard.KEY_SPACE:
     jump = true;
     break;
   }
  } else {
   switch (key) {
    case Keyboard.KEY_UP:
     forward = false;
     break;
    case Keyboard.KEY_DOWN:
     backward = false;
     break;
    case Keyboard.KEY_LEFT:
     left = false;
     break;
    case Keyboard.KEY_RIGHT:
     right = false;
     break;
    case Keyboard.KEY_SPACE:
     jump = false;
     break;
   }
  }
  return super.keyboardCallback(key, state);
 }

 @Override
 public boolean render_scene() {
  float delta_time = Math.min(move_clock.getTimeSeconds(), 0.1f);
  move_clock.reset();
  {
   final btMatrix3x3 xform = ghostObject.getWorldTransform().getBasis()
    .invert();
   final btVector3 forward_dir = new btVector3();
   xform.getRow(2, forward_dir);
   final btVector3 upDir = new btVector3();
   xform.getRow(1, upDir);
   final btVector3 sideways = new btVector3();
   xform.getRow(0, sideways);
   forward_dir.normalize();
   upDir.normalize();
   sideways.normalize();
   final btVector3 move = new btVector3();
   if (forward) {
    move.add(forward_dir);
   }
   if (backward) {
    move.sub(forward_dir);
   }
   if (move.lengthSquared() > 0) {
    move.normalize();
    move.scale(delta_time * 12.0f);
   }
   character.setWalkDirection(move);
   if (jump) {
    if (character.canJump()) {
     character.jump();
    }
    jump = false;
   }
   btVector3 angular = new btVector3();
   if (left) {
    angular.set(0, 2f, 0);
   }
   if (right) {
    angular.set(0, -2f, 0);
   }
   character.setAngularVelocity(angular);
  }
  boolean activity = super.render_scene();
  btTransform xform = ghostObject.getWorldTransform();
  btVector3 loc = xform.getOrigin();
  btCapsuleShape capsule = ((btCapsuleShape) ghostObject
   .getCollisionShape());
  loc.y += capsule.getHalfHeight() + capsule.getRadius() / 4.0f;
  xform.setOrigin(loc);
  float[] m = new float[16];
  xform.getOpenGLMatrix(m);
  glMultMatrix(put_matrix(m));
  GL11.glScalef(1.25f * capsule.getRadius(), 1.25f * capsule.getRadius(),
   1.25f * capsule
   .getRadius());
  draw_color();
  bind(HAT);
  HAT.draw();
  return activity;
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
  return "Cursor keys to move, space to jump.";
 }

}
