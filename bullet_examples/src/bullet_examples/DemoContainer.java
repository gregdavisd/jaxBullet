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
package bullet_examples;

import static Bullet.Collision.Broadphase.BroadphaseNativeTypes.*;
import Bullet.Collision.Broadphase.btAxisSweep3;
import Bullet.Collision.Broadphase.btBroadphaseInterface;
import Bullet.Collision.Broadphase.btDbvtBroadphase;
import Bullet.Collision.ClosestRayResultCallback;
import Bullet.Collision.Shape.btBoxShape;
import Bullet.Collision.Shape.btCapsuleShape;
import Bullet.Collision.Shape.btCollisionShape;
import Bullet.Collision.Shape.btCompoundShape;
import Bullet.Collision.Shape.btConeShape;
import Bullet.Collision.Shape.btConvexShape;
import Bullet.Collision.Shape.btCylinderShape;
import Bullet.Collision.Shape.btSphereShape;
import Bullet.Collision.Shape.btStaticPlaneShape;
import Bullet.Collision.Shape.btTriangleMeshShape;
import Bullet.Collision.btCollisionConfiguration;
import Bullet.Collision.Algorithm.btCollisionDispatcher;
import Bullet.Collision.Shape.btBvhTriangleMeshShape;
import Bullet.Collision.Shape.btIndexedMesh;
import Bullet.Collision.btCollisionObject;
import static Bullet.Collision.btCollisionObject.ISLAND_SLEEPING;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.Shape.btShapeHull;
import Bullet.Collision.Shape.btTriangleIndexVertexArray;
import Bullet.Dynamics.Constraint.btPoint2PointConstraint;
import Bullet.Dynamics.ConstraintSolver.btConstraintSolver;
import Bullet.Dynamics.btDynamicsWorld;
import Bullet.Dynamics.CollisionObjects.btRigidBody;
import Bullet.Dynamics.CollisionObjects.btRigidBodyConstructionInfo;
import static Bullet.Extras.btMinMax.btClamped;
import Bullet.LinearMath.btClock;
import Bullet.LinearMath.btDefaultMotionState;
import Bullet.LinearMath.btMatrix3x3;
import static Bullet.LinearMath.btScalar.FLT_MAX;
import Bullet.LinearMath.btTransform;
import Bullet.LinearMath.btVector3;
import static Bullet.LinearMath.btVector3.btPlaneSpace1;
import Bullet.LinearMath.btVector4;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape01Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape01Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape02Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape02Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape03Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape03Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape04Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape04Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape05Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape05Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape06Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape06Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape07Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape07Vtx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape08Idx;
import static bullet_examples.apps.benchmarks.LandscapeData.Landscape08Vtx;
import java.awt.Canvas;
import java.awt.Container;
import java.awt.event.MouseEvent;
import static java.awt.event.MouseEvent.BUTTON1;
import static java.awt.event.MouseEvent.BUTTON3;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.io.Serializable;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.AbstractMap.SimpleEntry;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import static java.util.stream.Collectors.mapping;
import static java.util.stream.Collectors.toList;
import static java.util.stream.Collectors.toMap;
import java.util.stream.IntStream;
import javax.swing.JFrame;
import javax.swing.JPanel;
import org.apache.commons.collections.primitives.ArrayFloatList;
import org.apache.commons.collections.primitives.ArrayIntList;
import org.lwjgl.BufferUtils;
import org.lwjgl.input.Keyboard;
import org.lwjgl.opengl.Display;
import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL12.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.util.glu.GLU.gluProject;
import static org.lwjgl.util.glu.GLU.gluUnProject;

/**
 *
 * @author Gregery Barton
 */
public abstract class DemoContainer implements PhysicsExample, MouseListener, MouseMotionListener,
 MouseWheelListener {

 private static final TriangleStripElement GL_BOX = new TriangleStripElement(
  new int[]{-1082130432, -1082130432, -1082130432, 0, -1082130432, -2147483648, 1065353216,
   -1082130432, -1082130432, 0, -1082130432, -2147483648, 1065353216, -1082130432, 1065353216, 0,
   -1082130432, -2147483648, -1082130432, -1082130432, 1065353216, 0, -1082130432, -2147483648,
   1065353216, -1082130432, 1065353216, 0, 0, 1065353216, 1065353216, 1065353216, 1065353216, 0, 0,
   1065353216, -1082130432, 1065353216, 1065353216, 0, 0, 1065353216, -1082130432, -1082130432,
   1065353216, 0, 0, 1065353216, -1082130432, -1082130432, 1065353216, -1082130432, 0, -2147483648,
   -1082130432, 1065353216, 1065353216, -1082130432, 0, -2147483648, -1082130432, 1065353216,
   -1082130432, -1082130432, 0, -2147483648, -1082130432, -1082130432, -1082130432, -1082130432, 0,
   -2147483648, -1082130432, -1082130432, -1082130432, 0, 0, -1082130432, -1082130432, 1065353216,
   -1082130432, 0, 0, -1082130432, 1065353216, 1065353216, -1082130432, 0, 0, -1082130432,
   1065353216, -1082130432, -1082130432, 0, 0, -1082130432, 1065353216, -1082130432, -1082130432,
   1065353216, 0, -2147483648, 1065353216, 1065353216, -1082130432, 1065353216, 0, -2147483648,
   1065353216, 1065353216, 1065353216, 1065353216, 0, -2147483648, 1065353216, -1082130432,
   1065353216, 1065353216, 0, -2147483648, 1065353216, 1065353216, -1082130432, 0, 1065353216,
   -2147483648, -1082130432, 1065353216, -1082130432, 0, 1065353216, -2147483648, -1082130432,
   1065353216, 1065353216, 0, 1065353216, -2147483648, 1065353216, 1065353216, 1065353216, 0,
   1065353216, -2147483648},
  new int[]{1, 2, 0, 3, 3, 5, 5, 6, 4, 7, 7, 9, 9, 10, 8, 11, 11, 13, 13, 14, 12, 15, 15, 17, 17, 18,
   16, 19, 19, 21, 21, 22, 20, 23});
 private static final TriangleStripElement GL_SPHERE = new TriangleStripElement(new int[]{0, 0,
  -1082130435, 0, 0, -1082130432, 0, 1053028117, -1083407522, 0, 1052661117, -1083333157, 1055687515,
  1053028117, -1085484145, 1055761883, 1052661117, -1085419620, 1056964605, 0, -1084378153,
  1056964096, 0, -1084378181, 0, 1060439283, -1087044365, 0, 1060173666, -1086785166, 0, 1064076126,
  -1094455530, 0, 1063880147, -1093549660, 1044639505, 1064076126, -1096175860, 1045545380,
  1063880147, -1095390869, 1052050672, 1060439283, -1088633744, 1052309874, 1060173666, -1088409280,
  0, -1083407522, -1094455531, 0, -1083603501, -1093549660, 0, -1087044365, -1087044365, 0,
  -1087309982, -1086785166, 1052050672, -1087044365, -1088633744, 1052309874, -1087309982,
  -1088409280, 1044639506, -1083407522, -1096175859, 1045545380, -1083603501, -1095390869, 0,
  -1094455528, -1083407522, 0, -1094822531, -1083333157, 1055687515, -1094455528, -1085484145,
  1055761883, -1094822531, -1085419620, 0, 1065353216, -2147483648, 0, 1065353216, -2147483648, 0,
  -1082130432, -2147483648, 0, -1082130432, -2147483648, 1058849902, -1087044365, -1095432971,
  1059074368, -1087309982, -1095173774, 1051307783, -1083407522, -1102844132, 1052092779,
  -1083603501, -1101938268, 1063105493, 0, -1090519039, 1063105467, 0, -1090519552, 1061999501,
  -1094455528, -1091796129, 1062064028, -1094822531, -1091721765, 1058849902, 1060439283,
  -1095432971, 1059074368, 1060173666, -1095173774, 1061999501, 1053028117, -1091796129, 1062064028,
  1052661117, -1091721765, 1051307782, 1064076126, -1102844133, 1052092779, 1063880147, -1101938268,
  1053028112, 1064076126, -2147483648, 1053933988, 1063880147, -2147483648, 1053028113, -1083407522,
  -2147483648, 1053933988, -1083603501, -2147483648, 1064076124, -1094455528, -2147483648,
  1064150491, -1094822531, -2147483648, 1060439280, -1087044365, -2147483648, 1060698482,
  -1087309982, -2147483648, 1064076124, 1053028117, -2147483648, 1064150491, 1052661117, -2147483648,
  1065353213, 0, -2147483648, 1065353216, 0, -2147483648, 1060439280, 1060439283, -2147483648,
  1060698482, 1060173666, -2147483648, 1061999501, -1094455528, 1055687512, 1062064028, -1094822531,
  1055761883, 1058849902, -1087044365, 1052050667, 1059074368, -1087309982, 1052309874, 1061999501,
  1053028117, 1055687512, 1062064028, 1052661117, 1055761883, 1063105492, 0, 1056964600, 1063105467,
  0, 1056964096, 1051307782, 1064076126, 1044639494, 1052092779, 1063880147, 1045545380, 1058849902,
  1060439283, 1052050667, 1059074368, 1060173666, 1052309874, 1051307782, -1083407522, 1044639495,
  1052092779, -1083603501, 1045545380, 1052050671, -1087044365, 1058849900, 1052309874, -1087309982,
  1059074368, 1044639503, -1083407522, 1051307777, 1045545380, -1083603501, 1052092779, 1056964603,
  0, 1063105490, 1056964096, 0, 1063105467, 1055687515, -1094455528, 1061999499, 1055761883,
  -1094822531, 1062064028, 1052050671, 1060439283, 1058849900, 1052309874, 1060173666, 1059074368,
  1055687515, 1053028117, 1061999499, 1055761883, 1052661117, 1062064028, 1044639504, 1064076126,
  1051307777, 1045545380, 1063880147, 1052092779, 0, 0, 1065353210, 0, 0, 1065353216, 0, -1094455528,
  1064076122, 0, -1094822531, 1064150491, 0, 1060439283, 1060439278, 0, 1060173666, 1060698482, 0,
  1053028117, 1064076122, 0, 1052661117, 1064150491, 0, 1064076126, 1053028107, 0, 1063880147,
  1053933988, 0, -1083407522, 1053028107, 0, -1083603501, 1053933988, 0, -1087044365, 1060439278, 0,
  -1087309982, 1060698482, -1102844143, -1083407522, 1051307776, -1101938268, -1083603501,
  1052092779, -1091796130, -1094455528, 1061999499, -1091721765, -1094822531, 1062064028,
  -1095432974, -1087044365, 1058849900, -1095173774, -1087309982, 1059074368, -1091796130,
  1053028117, 1061999499, -1091721765, 1052661117, 1062064028, -1090519042, 0, 1063105490,
  -1090519552, 0, 1063105467, -1102844144, 1064076126, 1051307776, -1101938268, 1063880147,
  1052092779, -1095432974, 1060439283, 1058849900, -1095173774, 1060173666, 1059074368, -1085484146,
  1053028117, 1055687510, -1085419620, 1052661117, 1055761883, -1084378155, 0, 1056964598,
  -1084378181, 0, 1056964096, -1096175866, 1064076126, 1044639493, -1095390869, 1063880147,
  1045545380, -1088633745, 1060439283, 1052050666, -1088409280, 1060173666, 1052309874, -1088633745,
  -1087044365, 1052050666, -1088409280, -1087309982, 1052309874, -1096175866, -1083407522,
  1044639493, -1095390869, -1083603501, 1045545380, -1085484146, -1094455528, 1055687510,
  -1085419620, -1094822531, 1055761883, -1087044367, -1087044365, -2147483648, -1086785166,
  -1087309982, -2147483648, -1094455536, -1083407522, -2147483648, -1093549660, -1083603501,
  -2147483648, -1082130435, 0, -2147483648, -1082130432, 0, -2147483648, -1083407523, -1094455528,
  -2147483648, -1083333157, -1094822531, -2147483648, -1087044367, 1060439283, -2147483648,
  -1086785166, 1060173666, -2147483648, -1083407523, 1053028117, -2147483648, -1083333157,
  1052661117, -2147483648, -1094455536, 1064076126, -2147483648, -1093549660, 1063880147,
  -2147483648, -1088633746, 1060439283, -1095432967, -1088409280, 1060173666, -1095173774,
  -1085484147, 1053028117, -1091796124, -1085419620, 1052661117, -1091721765, -1096175866,
  1064076126, -1102844133, -1095390869, 1063880147, -1101938268, -1096175866, -1083407522,
  -1102844133, -1095390869, -1083603501, -1101938268, -1085484147, -1094455528, -1091796124,
  -1085419620, -1094822531, -1091721765, -1088633746, -1087044365, -1095432967, -1088409280,
  -1087309982, -1095173774, -1084378157, 0, -1090519038, -1084378181, 0, -1090519552, -1102844145,
  -1083407522, -1096175861, -1101938268, -1083603501, -1095390869, -1091796133, -1094455528,
  -1085484144, -1091721765, -1094822531, -1085419620, -1095432977, -1087044365, -1088633742,
  -1095173774, -1087309982, -1088409280, -1091796133, 1053028117, -1085484144, -1091721765,
  1052661117, -1085419620, -1090519047, 0, -1084378154, -1090519552, 0, -1084378181, -1102844145,
  1064076126, -1096175861, -1101938268, 1063880147, -1095390869, -1095432977, 1060439283,
  -1088633742, -1095173774, 1060173666, -1088409280}, new int[]{9, 13, 10, 19, 16, 25, 26, 30, 31,
  40, 37, 45, 50, 52, 53, 64, 62, 68, 65, 76, 77, 80, 81, 12, 9, 12, 13, 3, 18, 21, 27, 29, 35, 34,
  43, 14, 48, 56, 46, 57, 47, 54, 44, 55, 45, 55, 52, 59, 64, 67, 68, 78, 76, 83, 80, 0, 12, 0, 3, 2,
  21, 20, 29, 23, 34, 23, 14, 22, 6, 7, 4, 1, 82, 83, 82, 78, 73, 67, 70, 59, 58, 55, 58, 54, 61, 57,
  60, 56, 60, 14, 71, 74, 69, 72, 70, 72, 73, 85, 82, 85, 4, 5, 6, 5, 14, 84, 74, 84, 72, 84, 85, 5,
  5, 45, 45, 40, 44, 39, 47, 42, 46, 41, 48, 41, 43, 41, 35, 32, 27, 28, 18, 19, 13, 13, 83, 83, 83,
  1, 0, 1, 2, 7, 20, 22, 23, 23, 19, 19, 19, 28, 25, 33, 30, 39, 40, 40, 28, 28, 28, 32, 33, 42, 39,
  39, 42, 42, 42, 32, 41, 41, 70, 70, 70, 58, 69, 61, 71, 60, 60, 10, 10, 8, 9, 79, 81, 75, 77, 66,
  65, 63, 62, 51, 53, 49, 50, 38, 37, 36, 31, 24, 26, 17, 16, 11, 10, 11, 8, 15, 79, 15, 75, 15, 66,
  15, 63, 15, 51, 15, 49, 15, 38, 15, 36, 15, 24, 15, 17, 11});
 private static final TriangleStripElement GL_CONE = new TriangleStripElement(new int[]{0,
  -1090519040, -1082130432, 0, -1095968422, -1083115550, 0, 1056964608, -2147483648, 0, 1065352704,
  -2147483648, 1056964608, -1090519040, -1084378153, 1055978978, -1095968422, -1085231711,
  1063105496, -1090519040, -1090519041, 1062251937, -1095968422, -1091504670, 1065353216,
  -1090519040, -2147483648, 1064368098, -1095968422, -2147483648, 1063105495, -1090519040,
  1056964609, 1062251937, -1095968422, 1055978978, 1056964609, -1090519040, 1063105495, 1055978978,
  -1095968422, 1062251937, 0, -1090519040, 1065353216, 0, -1095968422, 1064368098, -1090519047,
  -1090519040, 1063105497, -1091504670, -1095968422, 1062251937, -1084378156, -1090519040,
  1056964613, -1085231711, -1095968422, 1055978978, -1082130432, -1090519040, -2147483648,
  -1083115550, -1095968422, -2147483648, -1084378148, -1090519040, -1090519057, -1085231711,
  -1095968422, -1091504670, -1090519030, -1090519040, -1084378158, -1091504670, -1095968422,
  -1085231711}, new int[]{10, 12, 4, 2, 3, 1, 4, 5, 6, 1, 7, 8, 6, 8, 4, 8, 10, 9, 1, 8, 8, 1, 1, 2,
  0, 12, 1, 11, 10, 12});
 private static final TriangleStripElement GL_CYLINDER = new TriangleStripElement(new int[]{0,
  1065353216, -1082130432, 0, 1059535182, -1086205052, 1056964608, -1082130432, -1084378153,
  1052889476, -1087948466, -1087906480, 0, -1082130432, -1082130432, 0, -1087948466, -1086205052,
  1056964608, 1065353216, -1084378153, 1052889476, 1059535182, -1087906480, 1063105496, -1082130432,
  -1090519041, 1059577168, -1087948466, -1094594172, 1063105496, 1065353216, -1090519041, 1059577168,
  1059535182, -1094594172, 1065353216, -1082130432, -2147483648, 1061278596, -1087948466,
  -2147483648, 1065353216, 1065353216, -2147483648, 1061278596, 1059535182, -2147483648, 1063105495,
  -1082130432, 1056964609, 1059577168, -1087948466, 1052889476, 1063105495, 1065353216, 1056964609,
  1059577168, 1059535182, 1052889476, 1056964609, -1082130432, 1063105495, 1052889476, -1087948466,
  1059577168, 1056964609, 1065353216, 1063105495, 1052889476, 1059535182, 1059577168, 0, -1082130432,
  1065353216, 0, -1087948466, 1061278596, 0, 1065353216, 1065353216, 0, 1059535182, 1061278596,
  -1090519047, -1082130432, 1063105497, -1094594172, -1087948466, 1059577168, -1090519047,
  1065353216, 1063105497, -1094594172, 1059535182, 1059577168, -1084378156, -1082130432, 1056964613,
  -1087906480, -1087948466, 1052889476, -1084378156, 1065353216, 1056964613, -1087906480, 1059535182,
  1052889476, -1082130432, -1082130432, -2147483648, -1086205052, -1087948466, -2147483648,
  -1082130432, 1065353216, -2147483648, -1086205052, 1059535182, -2147483648, -1084378148,
  -1082130432, -1090519057, -1087906480, -1087948466, -1094594172, -1084378148, 1065353216,
  -1090519057, -1087906480, 1059535182, -1094594172, -1090519030, -1082130432, -1084378158,
  -1094594172, -1087948466, -1087906480, -1090519030, 1065353216, -1084378158, -1094594172,
  1059535182, -1087906480}, new int[]{5, 21, 13, 17, 15, 16, 14, 18, 6, 22, 1, 2, 0, 23, 21, 22, 20,
  18, 19, 17, 19, 21, 20, 20, 16, 16, 16, 17, 18, 18, 22, 22, 22, 23, 2, 2, 21, 21, 21, 5, 0, 3, 1,
  4, 6, 5, 7, 9, 8, 10, 6, 10, 14, 12, 13, 11, 9, 11, 10, 12, 12, 4, 4, 3, 5, 5, 6, 6, 6, 7, 8, 8, 5,
  5, 5, 13, 9, 9, 14, 14, 14, 13, 15});
 private static final TriangleStripElement GL_CAPSULE_CYLINDER = new TriangleStripElement(new int[]{
  0, 1065353216, 1065353215, 0, 1036358027, 1065274878, 0, -1082130431, 1065353218, 0, -1111125621,
  1065274878, 1056964608, -1082130429, 1063105499, 1056886270, -1111125621, 1063037369, 1056964604,
  1065353210, 1063105485, 1056886270, 1036358027, 1063037369, 1063105493, -1082130431, 1056964609,
  1063037369, -1111125621, 1056886270, 1063105492, 1065353210, 1056964591, 1063037369, 1036358027,
  1056886270, 1065353213, -1082130429, -2147483648, 1065274878, -1111125621, -2147483648, 1065353213,
  1065353210, -2147483648, 1065274878, 1036358027, -2147483648, 1063105492, -1082130429, -1090519049,
  1063037369, -1111125621, -1090597378, 1063105495, 1065353213, -1090519037, 1063037369, 1036358027,
  -1090597378, 1056964604, -1082130429, -1084378158, 1056886270, -1111125621, -1084446279,
  1056964609, 1065353212, -1084378151, 1056886270, 1036358027, -1084446279, 0, -1082130429,
  -1082130438, 0, -1111125621, -1082208770, 0, 1065353210, -1082130431, 0, 1036358027, -1082208770,
  -1090519043, -1082130429, -1084378158, -1090597378, -1111125621, -1084446279, -1090519048,
  1065353210, -1084378149, -1090597378, 1036358027, -1084446279, -1084378155, -1082130429,
  -1090519051, -1084446279, -1111125621, -1090597378, -1084378157, 1065353210, -1090519033,
  -1084446279, 1036358027, -1090597378, -1082130435, -1082130429, -2147483648, -1082208770,
  -1111125621, -2147483648, -1082130435, 1065353210, -2147483648, -1082208770, 1036358027,
  -2147483648, -1084378148, -1082130429, 1056964599, -1084446279, -1111125621, 1056886270,
  -1084378148, 1065353216, 1056964589, -1084446279, 1036358027, 1056886270, -1090519030, -1082130429,
  1063105494, -1090597378, -1111125621, 1063037369, -1090519030, 1065353216, 1063105488, -1090597378,
  1036358027, 1063037369},
  new int[]{2, 0, 1, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2,
   0});
 private static final TriangleStripElement GL_CAPSULE_CAP_BOTTOM = new TriangleStripElement(
  new int[]{0, -1082130431, 1065353218, 0, -1111125621, 1065274878, 0, -1078920249, 1064076127, 0,
   -1094822531, 1064150491, 1055687516, -1078920249, 1061999504, 1055761883, -1094822531, 1062064028,
   1056964608, -1082130429, 1063105499, 1056886270, -1111125621, 1063037369, 0, -1076198788,
   1060439285, 0, -1087309982, 1060698482, 0, -1074380366, 1053028125, 0, -1083603501, 1053933988,
   1044639507, -1074380366, 1051307795, 1045545380, -1083603501, 1052092779, 1052050673, -1076198788,
   1058849906, 1052309874, -1087309982, 1059074368, 0, -1073741822, -2147483648, 0, -1082130432,
   -2147483648, 1058849902, -1076198786, 1052050683, 1059074368, -1087309982, 1052309874, 1061999501,
   -1078920249, 1055687523, 1062064028, -1094822531, 1055761883, 1051307783, -1074380366, 1044639527,
   1052092779, -1083603501, 1045545380, 1063105493, -1082130431, 1056964609, 1063037369, -1111125621,
   1056886270, 1053028113, -1074380366, -2147483648, 1053933988, -1083603501, -2147483648,
   1064076124, -1078920247, -2147483648, 1064150491, -1094822531, -2147483648, 1065353213,
   -1082130429, -2147483648, 1065274878, -1111125621, -2147483648, 1060439280, -1076198786,
   -2147483648, 1060698482, -1087309982, -2147483648, 1061999501, -1078920247, -1091796139,
   1062064028, -1094822531, -1091721765, 1063105492, -1082130429, -1090519049, 1063037369,
   -1111125621, -1090597378, 1051307783, -1074380366, -1102844166, 1052092779, -1083603501,
   -1101938268, 1058849902, -1076198786, -1095432987, 1059074368, -1087309982, -1095173774,
   1052050672, -1076198786, -1088633750, 1052309874, -1087309982, -1088409280, 1055687516,
   -1078920247, -1085484150, 1055761883, -1094822531, -1085419620, 1044639506, -1074380364,
   -1096175877, 1045545380, -1083603501, -1095390869, 1056964604, -1082130429, -1084378158,
   1056886270, -1111125621, -1084446279, 0, -1076198786, -1087044372, 0, -1087309982, -1086785166, 0,
   -1078920247, -1083407527, 0, -1094822531, -1083333157, 0, -1074380364, -1094455547, 0,
   -1083603501, -1093549660, 0, -1082130429, -1082130438, 0, -1111125621, -1082208770, -1091796131,
   -1078920247, -1085484150, -1091721765, -1094822531, -1085419620, -1090519043, -1082130429,
   -1084378158, -1090597378, -1111125621, -1084446279, -1102844146, -1074380364, -1096175879,
   -1101938268, -1083603501, -1095390869, -1095432975, -1076198786, -1088633750, -1095173774,
   -1087309982, -1088409280, -1085484146, -1078920247, -1091796141, -1085419620, -1094822531,
   -1091721765, -1084378155, -1082130429, -1090519051, -1084446279, -1111125621, -1090597378,
   -1096175867, -1074380366, -1102844167, -1095390869, -1083603501, -1101938268, -1088633745,
   -1076198786, -1095432987, -1088409280, -1087309982, -1095173774, -1087044367, -1076198786,
   -2147483648, -1086785166, -1087309982, -2147483648, -1083407523, -1078920247, -2147483648,
   -1083333157, -1094822531, -2147483648, -1094455537, -1074380366, -2147483648, -1093549660,
   -1083603501, -2147483648, -1082130435, -1082130429, -2147483648, -1082208770, -1111125621,
   -2147483648, -1088633746, -1076198786, 1052050687, -1088409280, -1087309982, 1052309874,
   -1085484147, -1078920249, 1055687527, -1085419620, -1094822531, 1055761883, -1096175867,
   -1074380366, 1044639527, -1095390869, -1083603501, 1045545380, -1084378148, -1082130429,
   1056964599, -1084446279, -1111125621, 1056886270, -1091796134, -1078920249, 1061999505,
   -1091721765, -1094822531, 1062064028, -1090519030, -1082130429, 1063105494, -1090597378,
   -1111125621, 1063037369, -1102844147, -1074380366, 1051307793, -1101938268, -1083603501,
   1052092779, -1095432978, -1076198788, 1058849908, -1095173774, -1087309982, 1059074368},
  new int[]{0, 2, 3, 10, 12, 14, 15, 17, 18, 22, 24, 26, 28, 29, 30, 33, 34, 38, 40, 42, 44, 45, 46,
   1, 0, 1, 2, 7, 9, 11, 13, 8, 19, 23, 20, 21, 17, 21, 22, 25, 26, 32, 29, 36, 33, 37, 38, 41, 42,
   48, 45, 4, 1, 4, 7, 6, 11, 6, 8, 5, 47, 48, 47, 41, 43, 37, 39, 36, 35, 32, 31, 25, 27, 21, 27,
   23, 27, 8, 31, 8, 35, 8, 39, 8, 43, 47, 47, 17, 17, 14, 20, 16, 19, 16, 13, 16, 9, 10, 2, 2, 14,
   14, 14, 10, 16, 16, 6, 6, 6, 4, 5, 48});
 private static final TriangleStripElement GL_CAPSULE_CAP_TOP = new TriangleStripElement(
  new int[]{0, 1065353210, -1082130431, 0, 1036358027, -1082208770, 0, 1068563395, -1083407517, 0,
   1052661117, -1083333157, 1055687516, 1068563395, -1085484140, 1055761883, 1052661117,
   -1085419620, 1056964609, 1065353212, -1084378151, 1056886270, 1036358027, -1084446279, 0,
   1071284856, -1087044360, 0, 1060173666, -1086785166, 0, 1073103278, -1094455521, 0, 1063880147,
   -1093549660, 1044639507, 1073103278, -1096175851, 1045545380, 1063880147, -1095390869,
   1052050673, 1071284856, -1088633739, 1052309874, 1060173666, -1088409280, 0, 1073741822,
   -2147483648, 0, 1065353216, -2147483648, 1058849902, 1071284856, -1095432962, 1059074368,
   1060173666, -1095173774, 1061999501, 1068563395, -1091796120, 1062064028, 1052661117,
   -1091721765, 1051307783, 1073103278, -1102844115, 1052092779, 1063880147, -1101938268,
   1063105495, 1065353213, -1090519037, 1063037369, 1036358027, -1090597378, 1053028113, 1073103278,
   -2147483648, 1053933988, 1063880147, -2147483648, 1064076124, 1068563395, -2147483648,
   1064150491, 1052661117, -2147483648, 1065353213, 1065353210, -2147483648, 1065274878, 1036358027,
   -2147483648, 1060439280, 1071284856, -2147483648, 1060698482, 1060173666, -2147483648,
   1061999501, 1068563395, 1055687503, 1062064028, 1052661117, 1055761883, 1063105492, 1065353210,
   1056964591, 1063037369, 1036358027, 1056886270, 1051307783, 1073103278, 1044639476, 1052092779,
   1063880147, 1045545380, 1058849902, 1071284856, 1052050658, 1059074368, 1060173666, 1052309874,
   1052050672, 1071284856, 1058849895, 1052309874, 1060173666, 1059074368, 1055687516, 1068563395,
   1061999494, 1055761883, 1052661117, 1062064028, 1044639506, 1073103278, 1051307768, 1045545380,
   1063880147, 1052092779, 1056964604, 1065353210, 1063105485, 1056886270, 1036358027, 1063037369,
   0, 1071284856, 1060439273, 0, 1060173666, 1060698482, 0, 1068563395, 1064076117, 0, 1052661117,
   1064150491, 0, 1073103278, 1053028098, 0, 1063880147, 1053933988, 0, 1065353216, 1065353215, 0,
   1036358027, 1065274878, -1091796131, 1068563395, 1061999494, -1091721765, 1052661117, 1062064028,
   -1090519030, 1065353216, 1063105488, -1090597378, 1036358027, 1063037369, -1102844146,
   1073103278, 1051307767, -1101938268, 1063880147, 1052092779, -1095432975, 1071284856, 1058849895,
   -1095173774, 1060173666, 1059074368, -1085484146, 1068563395, 1055687501, -1085419620,
   1052661117, 1055761883, -1084378148, 1065353216, 1056964589, -1084446279, 1036358027, 1056886270,
   -1096175867, 1073103278, 1044639475, -1095390869, 1063880147, 1045545380, -1088633745,
   1071284856, 1052050657, -1088409280, 1060173666, 1052309874, -1087044367, 1071284856,
   -2147483648, -1086785166, 1060173666, -2147483648, -1083407523, 1068563395, -2147483648,
   -1083333157, 1052661117, -2147483648, -1094455537, 1073103278, -2147483648, -1093549660,
   1063880147, -2147483648, -1082130435, 1065353210, -2147483648, -1082208770, 1036358027,
   -2147483648, -1088633746, 1071284856, -1095432958, -1088409280, 1060173666, -1095173774,
   -1085484147, 1068563395, -1091796115, -1085419620, 1052661117, -1091721765, -1096175867,
   1073103278, -1102844115, -1095390869, 1063880147, -1101938268, -1084378157, 1065353210,
   -1090519033, -1084446279, 1036358027, -1090597378, -1091796134, 1068563395, -1085484139,
   -1091721765, 1052661117, -1085419620, -1090519048, 1065353210, -1084378149, -1090597378,
   1036358027, -1084446279, -1102844147, 1073103278, -1096175852, -1101938268, 1063880147,
   -1095390869, -1095432978, 1071284856, -1088633737, -1095173774, 1060173666, -1088409280},
  new int[]{0, 2, 3, 10, 12, 14, 15, 17, 18, 22, 24, 26, 28, 29, 30, 33, 34, 38, 40, 42, 44, 45, 46,
   1, 0, 1, 2, 7, 9, 11, 13, 8, 19, 23, 20, 21, 17, 21, 22, 25, 26, 32, 29, 36, 33, 37, 38, 41, 42,
   48, 45, 4, 1, 4, 7, 6, 11, 6, 8, 5, 47, 48, 47, 41, 43, 37, 39, 36, 35, 32, 31, 25, 27, 21, 27,
   23, 27, 8, 31, 8, 35, 8, 39, 8, 43, 47, 47, 17, 17, 14, 20, 16, 19, 16, 13, 16, 9, 10, 2, 2, 14,
   14, 14, 10, 16, 16, 6, 6, 6, 4, 5, 48});
 private static final Camera camera = new Camera();
 private static boolean forward_pressed;
 private static boolean backward_pressed;
 private static boolean right_pressed;
 private static boolean left_pressed;
 private static boolean quick_pressed;
 protected btDynamicsWorld world;
 protected btIDebugDraw debug_draw;
 protected btCollisionConfiguration collision_configuration;
 protected btCollisionDispatcher dispatcher;
 protected btBroadphaseInterface broadphase;
 protected btConstraintSolver constraint_solver;
 private btClock physics_clock;
 float[] color_cache = new float[4];
 private int enabled_light = 0;
 private final FloatBuffer matrix_buffer = BufferUtils.createFloatBuffer(16);
 private final FloatBuffer four_buffer = BufferUtils.createFloatBuffer(16);
 private GLDrawElements bound = null;
 btPoint2PointConstraint mouse_constraint;
 private float mouse_length;
 private int up_axis = 0;
 private static int mouse_x;
 private static int mouse_y;
 private static final Map<Object, GLDrawElements> shapes = new HashMap<>();

 public DemoContainer() {
  for (Map.Entry<Object, GLDrawElements> shape : shapes.entrySet()) {
   shape.getValue().destroy();
  }
  shapes.clear();
  init();
 }

 public void setUpAxis(int axis) {
  up_axis = axis;
 }

 final void init() {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHT1);
  glEnable(GL_LIGHT2);
  glEnable(GL_LIGHT3);
  glLight(GL_LIGHT0, GL_AMBIENT, put_4f(new float[]{148 / 255.0f, 148 / 255.0f, 246 / 255.0f, 1.0f}));
  glLight(GL_LIGHT0, GL_DIFFUSE, put_4f(new float[]{148 / 255.0f, 148 / 255.0f, 246 / 255.0f, 1.0f}));
  glLight(GL_LIGHT0, GL_SPECULAR, put_4f(new float[]{1.0f, 1.0f, 1.0f, 1.0f}));
  glLight(GL_LIGHT0, GL_POSITION, put_4f(new float[]{10.0f, 10.0f, 10.0f, 1.0f}));
  glLight(GL_LIGHT1, GL_AMBIENT, put_4f(new float[]{0 / 255.0f, 246 / 255.0f, 0 / 255.0f, 1.0f}));
  glLight(GL_LIGHT1, GL_DIFFUSE, put_4f(new float[]{0 / 255.0f, 246 / 255.0f, 0 / 255.0f, 1.0f}));
  glLight(GL_LIGHT1, GL_SPECULAR, put_4f(new float[]{1.0f, 1.0f, 1.0f, 1.0f}));
  glLight(GL_LIGHT1, GL_POSITION, put_4f(new float[]{10.0f, 10.0f, 10.0f, 1.0f}));
  glLight(GL_LIGHT2, GL_AMBIENT, put_4f(new float[]{74 / 255.0f, 74 / 255.0f, 246 / 255.0f, 1.0f}));
  glLight(GL_LIGHT2, GL_DIFFUSE, put_4f(new float[]{74 / 255.0f, 74 / 255.0f, 246 / 255.0f, 1.0f}));
  glLight(GL_LIGHT2, GL_SPECULAR, put_4f(new float[]{1.0f, 1.0f, 1.0f, 1.0f}));
  glLight(GL_LIGHT2, GL_POSITION, put_4f(new float[]{10.0f, 10.0f, 10.0f, 1.0f}));
  glLight(GL_LIGHT3, GL_AMBIENT, put_4f(new float[]{0 / 255.0f, 246 / 255.0f, 246 / 255.0f, 1.0f}));
  glLight(GL_LIGHT3, GL_DIFFUSE, put_4f(new float[]{0 / 255.0f, 246 / 255.0f, 246 / 255.0f, 1.0f}));
  glLight(GL_LIGHT3, GL_SPECULAR, put_4f(new float[]{1.0f, 1.0f, 1.0f, 1.0f}));
  glLight(GL_LIGHT3, GL_POSITION, put_4f(new float[]{10.0f, 10.0f, 10.0f, 1.0f}));
  glEnable(GL_LIGHT0);
  glDisable(GL_LIGHT1);
  glDisable(GL_LIGHT2);
  glDisable(GL_LIGHT3);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glEnable(GL_RESCALE_NORMAL);
  glClearColor(178 / 255.0f, 178 / 255.0f, 204 / 255.0f, 1);
 }

 abstract protected void initWorld(String broadphase_class);

 private void disable_vertex_arrays() {
  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_NORMAL_ARRAY);
  bound = null;
 }

 private void enable_vertex_arrays() {
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_NORMAL_ARRAY);
 }

 public btDynamicsWorld world() {
  return world;
 }

 protected btRigidBody createRigidBody(float mass, final btTransform startTransform,
  btCollisionShape shape) {
  return createRigidBody(mass, startTransform, shape, new btVector4(1, 0, 0, 1));
 }

 protected btRigidBody createRigidBody(float mass, final btTransform startTransform,
  btCollisionShape shape,
  btVector4 color) {
  assert (shape == null || shape.getShapeType() != INVALID_SHAPE_PROXYTYPE);
  boolean isDynamic = (mass != 0.f);
  final btVector3 localInertia = new btVector3();
  if (isDynamic && shape != null) {
   shape.calculateLocalInertia(mass, localInertia);
  }
  btDefaultMotionState myMotionState = new btDefaultMotionState(startTransform);
  btRigidBodyConstructionInfo cInfo = new btRigidBodyConstructionInfo(mass, myMotionState, shape,
   localInertia);
  btRigidBody body = new btRigidBody(cInfo);
  body.setUserIndex(-1);
  world.addRigidBody(body);
  return body;
 }

 @Override
 public abstract void initPhysics();

 public boolean step_physics(float cap_frametime, float rate) {
  return step_physics(cap_frametime, rate, 4);
 }

 public boolean step_physics(float cap_frametime, float rate, int max_substeps) {
  if (physics_clock == null) {
   physics_clock = new btClock();
  }
  long time_acc = physics_clock.getTimeMicroseconds();
  if (time_acc > ((cap_frametime) * 1e6f)) {
   physics_clock.reset();
   float fixed = btClamped(cap_frametime, 1.0f / 120.0f, 1.0f / 30.0f);
   float step_time = rate * (float) time_acc / 1e6f;
   int sub_steps = ((int) (step_time / fixed));
   if (step_time > ((sub_steps * (fixed + 0.001f)))) {
    ++sub_steps;
   }
   if (sub_steps > max_substeps) {
    step_time -= (sub_steps - max_substeps) * fixed;
    sub_steps = max_substeps;
   }
   world.stepSimulation(step_time, sub_steps, fixed);
   return true;
  }
  return false;
 }

 protected void draw_color() {
  glDisable(GL_LIGHT0 + enabled_light);
  enabled_light = (enabled_light + 1) % 4;
  glEnable(GL_LIGHT0 + enabled_light);
 }

 public boolean render_scene() {
  glViewport(Display.getX(), Display.getY(), Display.getWidth(), Display.getHeight());
  background_color();
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  if (world() == null) {
   return false;
  }
  camera_matrices();
  process_keyboard_events();
  camera.animate_camera(!quick_pressed ? 20.0f : 60.0f, forward_pressed, backward_pressed,
   left_pressed, right_pressed);
  glDisable(GL_LIGHTING);
  boolean activity = false;
  int debug = world().getDebugDrawer().getDebugMode() | getDebugMode();
  if (debug != 0) {
   world().getDebugDrawer().setDebugMode(debug);
disable_vertex_arrays();
   activity = world.debugDrawWorld();
  }
  if ((world.getDebugDrawer().getDebugMode() & btIDebugDraw.DBG_DrawWireframe) == 0) {
   glEnable(GL_LIGHTING);
     enable_vertex_arrays();
   return draw_collision_shapes();
  } else {
   return activity;
  }
 }

 private void process_keyboard_events() {
  while (Keyboard.next()) {
   int key = Keyboard.getEventKey();
   switch (key) {
    case Keyboard.KEY_W:
     forward_pressed = Keyboard.getEventKeyState();
     break;
    case Keyboard.KEY_S:
     backward_pressed = Keyboard.getEventKeyState();
     break;
    case Keyboard.KEY_A:
     left_pressed = Keyboard.getEventKeyState();
     break;
    case Keyboard.KEY_D:
     right_pressed = Keyboard.getEventKeyState();
     break;
    case Keyboard.KEY_LSHIFT:
    case Keyboard.KEY_RSHIFT:
     quick_pressed = Keyboard.getEventKeyState();
     break;
   }
     keyboardCallback(key,Keyboard.getEventKeyState()?1:0);
  }
 }

 protected JFrame frame() {
  Container parent = Display.getParent().getParent();
  while (!(parent instanceof JFrame)) {
   if (parent != null) {
    parent = parent.getParent();
   } else {
    return null;
   }
  }
  return (JFrame) parent;
 }

 protected Canvas canvas() {
  return Display.getParent();
 }

 private void drop_mouse_constraint() {
  mouse_constraint.getRigidBodyA().forceActivationState(btCollisionObject.ACTIVE_TAG);
  world.removeConstraint(mouse_constraint);
  mouse_constraint = null;
 }

 btVector3 unproject(int x, int y, float z) {
  IntBuffer viewport_buffer = BufferUtils.createIntBuffer(16);
  glGetInteger(GL_VIEWPORT, viewport_buffer);
  FloatBuffer modelview_buffer = BufferUtils.createFloatBuffer(16);
  glGetFloat(GL_MODELVIEW_MATRIX, modelview_buffer);
  FloatBuffer projection_buffer = BufferUtils.createFloatBuffer(16);
  glGetFloat(GL_PROJECTION_MATRIX, projection_buffer);
  float winX = x;
  float winY = y;
  float winZ = z;
  FloatBuffer view_pos = BufferUtils.createFloatBuffer(16);
  if (gluUnProject(winX, winY, winZ, modelview_buffer, projection_buffer, viewport_buffer, view_pos)) {
   final btVector3 p = new btVector3(view_pos.get(0), view_pos.get(1), view_pos.get(2));
   return p;
  } else {
   return new btVector3();
  }
 }

 protected btVector3 project(float x, float y, float z) {
  IntBuffer viewport_buffer = BufferUtils.createIntBuffer(16);
  glGetInteger(GL_VIEWPORT, viewport_buffer);
  FloatBuffer modelview_buffer = BufferUtils.createFloatBuffer(16);
  glGetFloat(GL_MODELVIEW_MATRIX, modelview_buffer);
  FloatBuffer projection_buffer = BufferUtils.createFloatBuffer(16);
  glGetFloat(GL_PROJECTION_MATRIX, projection_buffer);
  FloatBuffer view_pos = BufferUtils.createFloatBuffer(16);
  if (gluProject(x, y, z, modelview_buffer, projection_buffer, viewport_buffer, view_pos)) {
   final btVector3 pos = new btVector3(view_pos.get(0), view_pos.get(1), view_pos.get(2));
   pos.setY(canvas().getHeight() - pos.getY());
   return pos;
  } else {
   return new btVector3();
  }
 }

 private boolean draw_collision_shapes() {
  boolean all_sleeping = true;

 for (btCollisionObject collisionObject : world.getCollisionObjectArray()) {
     if (collisionObject.getActivationState() != ISLAND_SLEEPING) {
    all_sleeping = false;
   }
   btRigidBody body = btRigidBody.upcast(collisionObject);
   if (body != null) {

   final btTransform object_trans = new btTransform();
   if (body.getMotionState() != null) {
    btDefaultMotionState myMotionState = (btDefaultMotionState) body.getMotionState();
    object_trans.set(myMotionState.m_graphicsWorldTrans);
   } else {
    object_trans.set(collisionObject.getWorldTransformPtr());
   }
   draw_color();
   draw_shape(object_trans, collisionObject.getCollisionShape());
  }
   else
   {
    draw_color();
     draw_shape(collisionObject.getWorldTransform(),collisionObject.getCollisionShape());
   }
 }
 
  return !all_sleeping;
 }

 private void rotate_to_up_axis(int axis) {
  switch (axis) {
   case 0:
    glRotatef(90, 0, 0, -1);
    break;
   case 1:
    break;
   case 2:
    glRotatef(90, 1, 0, 0);
    break;
   default:
    assert (false);
  }
 }

 private void rotate_from_up_axis(int axis) {
  switch (axis) {
   case 0:
    glRotatef(-90, 0, 0, -1);
    break;
   case 1:
    break;
   case 2:
    glRotatef(-90, 1, 0, 0);
    break;
   default:
    assert (false);
  }
 }

 private void rotate_to_up_axis(int axis, final btVector3 vec) {
  switch (axis) {
   case 0:
    new btMatrix3x3().rotZ((float) Math.toRadians(-90f)).transform(vec);
    break;
   case 1:
    break;
   case 2:
    new btMatrix3x3().rotX((float) Math.toRadians(90f)).transform(vec);
    break;
   default:
    assert (false);
  }
 }

 protected FloatBuffer put_matrix(float[] matrix) {
  matrix_buffer.put(matrix);
  matrix_buffer.rewind();
  return matrix_buffer;
 }

 private FloatBuffer put_4f(float[] matrix) {
  four_buffer.put(matrix);
  four_buffer.rewind();
  return four_buffer;
 }

 protected void draw_shape(final btTransform object_trans, btCollisionShape collisionShape) {
  glPushMatrix();
  {
   float[] matrix = new float[16];
   object_trans.getOpenGLMatrix(matrix);
   glMultMatrix(put_matrix(matrix));
  }
  switch (collisionShape.getShapeType()) {
   case BOX_SHAPE_PROXYTYPE: {
    btBoxShape box = (btBoxShape) collisionShape;
    final btVector3 half_extents = box.getHalfExtentsWithMargin();
    glScalef(half_extents.x, half_extents.y, half_extents.z);
    bind(GL_BOX);
    GL_BOX.draw();
   }
   break;
   case TRIANGLE_SHAPE_PROXYTYPE: {
    System.out.println("drawing  TRIANGLE_SHAPE_PROXYTYPE not");
   }
   break;
   case TETRAHEDRAL_SHAPE_PROXYTYPE: {
    System.out.println("drawing  TETRAHEDRAL_SHAPE_PROXYTYPE not");
   }
   break;
   case CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE: {
    System.out.println("drawing  CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE not");
   }
   break;
   case CONVEX_HULL_SHAPE_PROXYTYPE:
    draw_novel_shape(collisionShape);
    break;
   case CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE: {
    System.out.println("drawing  CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE not");
   }
   break;
   case CUSTOM_POLYHEDRAL_SHAPE_TYPE: {
    System.out.println("drawing  CUSTOM_POLYHEDRAL_SHAPE_TYPE not");
   }
   break;
   case IMPLICIT_CONVEX_SHAPES_START_HERE: {
    System.out.println("drawing  IMPLICIT_CONVEX_SHAPES_START_HERE not");
   }
   break;
   case SPHERE_SHAPE_PROXYTYPE: {
    btSphereShape sphere = (btSphereShape) collisionShape;
    final float radius = sphere.getRadius();
    glScalef(radius, radius, radius);
    bind(GL_SPHERE);
    GL_SPHERE.draw();
   }
   break;
   case MULTI_SPHERE_SHAPE_PROXYTYPE: {
    System.out.println("drawing  MULTI_SPHERE_SHAPE_PROXYTYPE not");
   }
   break;
   case CAPSULE_SHAPE_PROXYTYPE: {
    btCapsuleShape capsule = (btCapsuleShape) collisionShape;
    rotate_to_up_axis(capsule.getUpAxis());
    glPushMatrix();
    glScalef(capsule.getRadius(), capsule.getHalfHeight(), capsule.getRadius());
    bind(GL_CAPSULE_CYLINDER);
    GL_CAPSULE_CYLINDER.draw();
    glPopMatrix();
    float cap_offset = capsule.getHalfHeight() - capsule.getRadius();
    glPushMatrix();
    glTranslatef(0, -cap_offset, 0);
    glScalef(capsule.getRadius(), capsule.getRadius(), capsule.getRadius());
    bind(GL_CAPSULE_CAP_BOTTOM);
    GL_CAPSULE_CAP_BOTTOM.draw();
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0, cap_offset, 0);
    glScalef(capsule.getRadius(), capsule.getRadius(), capsule.getRadius());
    bind(GL_CAPSULE_CAP_TOP);
    GL_CAPSULE_CAP_TOP.draw();
    glPopMatrix();
   }
   break;
   case CONE_SHAPE_PROXYTYPE: {
    btConeShape cone = (btConeShape) collisionShape;
    final btVector3 half_extents = new btVector3(cone.getRadius() + cone.getMargin(), cone
     .getHeight() + 2.0f * cone.getMargin(), cone
     .getRadius() + cone.getMargin());
    rotate_to_up_axis(cone.getConeUpIndex());
    glScalef(half_extents.x, half_extents.y, half_extents.z);
    bind(GL_CONE);
    GL_CONE.draw();
   }
   break;
   case CONVEX_SHAPE_PROXYTYPE: {
    System.out.println("drawing  CONVEX_SHAPE_PROXYTYPE not");
   }
   break;
   case CYLINDER_SHAPE_PROXYTYPE: {
    btCylinderShape cylinder = (btCylinderShape) collisionShape;
    final btVector3 half_extents = cylinder.getHalfExtentsWithMargin();
    glScalef(half_extents.x, half_extents.y, half_extents.z);
    rotate_to_up_axis(cylinder.getUpAxis());
    bind(GL_CYLINDER);
    GL_CYLINDER.draw();
   }
   break;
   case UNIFORM_SCALING_SHAPE_PROXYTYPE: {
    System.out.println("drawing  UNIFORM_SCALING_SHAPE_PROXYTYPE not");
   }
   break;
   case MINKOWSKI_SUM_SHAPE_PROXYTYPE: {
    System.out.println("drawing  MINKOWSKI_SUM_SHAPE_PROXYTYPE not");
   }
   break;
   case MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE: {
    System.out.println("drawing  MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE not");
   }
   break;
   case BOX_2D_SHAPE_PROXYTYPE: {
    System.out.println("drawing  BOX_2D_SHAPE_PROXYTYPE not");
   }
   break;
   case CONVEX_2D_SHAPE_PROXYTYPE: {
    System.out.println("drawing  CONVEX_2D_SHAPE_PROXYTYPE not");
   }
   break;
   case CUSTOM_CONVEX_SHAPE_TYPE: {
    System.out.println("drawing  CUSTOM_CONVEX_SHAPE_TYPE not");
   }
   break;
   case CONCAVE_SHAPES_START_HERE: {
    System.out.println("drawing  CONCAVE_SHAPES_START_HERE not");
   }
   break;
   case TRIANGLE_MESH_SHAPE_PROXYTYPE: {
    draw_novel_shape(collisionShape);
   }
   break;
   case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE: {
    System.out.println("drawing  SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE not");
   }
   break;
   case FAST_CONCAVE_MESH_PROXYTYPE: {
    System.out.println("drawing  FAST_CONCAVE_MESH_PROXYTYPE not");
   }
   break;
   case TERRAIN_SHAPE_PROXYTYPE: {
    System.out.println("drawing  TERRAIN_SHAPE_PROXYTYPE not");
   }
   break;
   case GIMPACT_SHAPE_PROXYTYPE: {
    System.out.println("drawing  GIMPACT_SHAPE_PROXYTYPE not");
   }
   break;
   case MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE: {
    System.out.println("drawing  MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE not");
   }
   break;
   case EMPTY_SHAPE_PROXYTYPE: {
    System.out.println("drawing  EMPTY_SHAPE_PROXYTYPE not");
   }
   break;
   case STATIC_PLANE_PROXYTYPE: {
    btStaticPlaneShape plane = (btStaticPlaneShape) collisionShape;
    float d = plane.getPlaneConstant();
    final btVector3 n = plane.getPlaneNormal();
    final btVector3 origin = new btVector3(n).scale(d);
    final btVector3 p = new btVector3();
    final btVector3 q = new btVector3();
    btPlaneSpace1(n, p, q);
    final btMatrix3x3 rot = new btMatrix3x3();
    rot.setColumn(0, p);
    rot.setColumn(1, n);
    rot.setColumn(2, q);
    final btTransform trans = new btTransform(rot, origin);
    float[] matrix = new float[16];
    trans.getOpenGLMatrix(matrix);
    glMultMatrix(put_matrix(matrix));
    glScalef(100, 0, 100);
    bind(GL_BOX);
    GL_BOX.draw();
   }
   break;
   case CUSTOM_CONCAVE_SHAPE_TYPE: {
    System.out.println("drawing  CUSTOM_CONCAVE_SHAPE_TYPE not");
   }
   break;
   case CONCAVE_SHAPES_END_HERE: {
    System.out.println("drawing  CONCAVE_SHAPES_END_HERE not");
   }
   break;
   case COMPOUND_SHAPE_PROXYTYPE: {
    btCompoundShape compoundShape = (btCompoundShape) collisionShape;
    for (int i = 0; i < compoundShape.getNumChildShapes(); ++i) {
     btCollisionShape child_shape = compoundShape.getChildShape(i);
     final btTransform childTrans = compoundShape.getChildTransform(i);
     draw_shape(childTrans, child_shape);
    }
    compoundShape.getNumChildShapes();
   }
   break;
   case SOFTBODY_SHAPE_PROXYTYPE: {
    System.out.println("drawing  SOFTBODY_SHAPE_PROXYTYPE not");
   }
   break;
   case HFFLUID_SHAPE_PROXYTYPE: {
    System.out.println("drawing  HFFLUID_SHAPE_PROXYTYPE not");
   }
   break;
   case HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE: {
    System.out.println("drawing  HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE not");
   }
   break;
   case INVALID_SHAPE_PROXYTYPE: {
    System.out.println("drawing  INVALID_SHAPE_PROXYTYPE not");
   }
   break;
   case MAX_BROADPHASE_COLLISION_TYPES: {
    System.out.println("drawing  MAX_BROADPHASE_COLLISION_TYPES not");
   }
   break;
  }
  glPopMatrix();
 }

 private static class ShapeKey implements Serializable
 {
  
 }
 private void draw_novel_shape(btCollisionShape collisionShape) {
 if (collisionShape.getUserPointer()==null){
  collisionShape.setUserPointer(new ShapeKey());
 }
  GLDrawElements shape = shapes.get(collisionShape.getUserPointer());
  if (shape == null) {
   shape = shape_to_mesh(collisionShape);
   if (shape != null) {
    shapes.put(collisionShape.getUserPointer(), shape);
   }
  }
  if (shape != null) {
   bind(shape);
   shape.draw();
  }
 }

 protected void bind(GLDrawElements triangles) {
  if (bound != triangles) {
   triangles.bind();
   bound = triangles;
  }
 }

 private void background_color() {
  glDisable(GL_LIGHT0 + enabled_light);
  enabled_light = 3;
 }

 abstract public void resetCamera();

 private void camera_matrices() {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float aspect = Math.max(Math.min((float) Display.getWidth() / (float) Display.getHeight(), 4.0f),
   1 / 4.0f);
  glFrustum(-0.2 * aspect, 0.2 * aspect, -0.2, 0.2, 1.0, 10000.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  final btTransform camera_matrix = camera.get_matrix();
  float[] m = new float[16];
  camera_matrix.getOpenGLMatrix(m);
  glMultMatrix(put_matrix(m));
  rotate_from_up_axis(up_axis);
 }

 public void set_debug_flag(boolean flag, int bit_flag) {
  int debug_mode = world.getDebugDrawer().getDebugMode();
  if (flag) {
   debug_mode |= bit_flag;
  } else {
   debug_mode &= ~bit_flag;
  }
  set_debug_mode(debug_mode);
 }

 public int set_debug_mode(int debug_mode) {
  world.getDebugDrawer().setDebugMode(debug_mode | getDebugMode());
  return world.getDebugDrawer().getDebugMode();
 }

 protected abstract int getDebugMode();

 public boolean keyboardCallback(int key, int state) {
  return false;
 }

 protected Camera camera() {
  return camera;
 }

 protected abstract JPanel getParams();

 public abstract String get_description();

 @Override
 public void mouseDragged(MouseEvent e) {
  mouseMoved(e);
 }

 @Override
 public void mouseMoved(MouseEvent e) {
  if (mouse_constraint != null) {
   camera_matrices();
   final btVector3 ray_from = camera.eye();
   rotate_to_up_axis(up_axis, ray_from);
   final btVector3 ray_to = unproject(e.getX(), canvas().getHeight() - e.getY(), 1);
   final btVector3 ray_dir = new btVector3(ray_to).sub(ray_from).normalize();
   final btVector3 new_pos = new btVector3().scaleAdd(mouse_length, ray_dir, ray_from);
   mouse_constraint.setPivotB(new_pos);
  }
  OpenGLFrame frame = (OpenGLFrame) frame();
  if ((frame != null) && frame.is_grabbed()) {
   float dx = mouse_x - e.getX();
   float dy = mouse_y - e.getY();
   camera.look_around(-dx / 1500.0f, dy / 2000.0f);
  }
  mouse_x = e.getX();
  mouse_y = e.getY();
 }

 @Override
 public void mouseClicked(MouseEvent e) {
 }

 @Override
 public void mousePressed(MouseEvent e) {
  if (e.getButton() == BUTTON1) {
   OpenGLFrame frame = (OpenGLFrame) frame();
   if (frame != null) {
    frame.set_grabbed(!frame.is_grabbed());
   }
  } else if (e.getButton() == BUTTON3) {
   camera_matrices();
   final btVector3 ray_from = camera.eye();
   rotate_to_up_axis(up_axis, ray_from);
   final btVector3 ray_to = unproject(e.getX(), canvas().getHeight() - e.getY(), 1);
   ClosestRayResultCallback callback = new ClosestRayResultCallback(ray_from, ray_to);
   world.rayTest(ray_from, ray_to, callback);
   if (callback.hasHit()) {
    btRigidBody body = btRigidBody.upcast(callback.m_collisionObject);
    if (body != null) {
     body.setActivationState(btCollisionObject.DISABLE_DEACTIVATION);
     final btVector3 hit = new btVector3(callback.m_hitPointWorld);
     final btTransform body_trans = body.getCenterOfMassTransform().invert();
     body_trans.transform(hit);
     if (mouse_constraint != null) {
      drop_mouse_constraint();
     }
     btPoint2PointConstraint p2p = new btPoint2PointConstraint(body, hit);
     p2p.m_setting.m_impulseClamp = 3;
     p2p.m_setting.m_tau = 0.1f;
     world.addConstraint(p2p);
     mouse_constraint = p2p;
     mouse_length = new btVector3(callback.m_hitPointWorld).sub(ray_from).length();
    }
   }
  }
 }

 @Override
 public void mouseReleased(MouseEvent e) {
  if (e.getButton() == BUTTON3) {
   if (mouse_constraint != null) {
    drop_mouse_constraint();
    mouse_constraint = null;
   }
  }
 }

 @Override
 public void mouseEntered(MouseEvent e) {
 }

 @Override
 public void mouseExited(MouseEvent e) {
 }

 @Override
 public void mouseWheelMoved(MouseWheelEvent e) {
 }

 protected void create_broadphase(String broadphase_class) {
  switch (broadphase_class) {
   case "btDbvtBroadphase":
    broadphase = new btDbvtBroadphase();
    break;
   case "btAxisSweep3":
    final btVector3 worldAabbMin = new btVector3(-1000, -1000, -1000);
    final btVector3 worldAabbMax = new btVector3(1000, 1000, 1000);
    broadphase = new btAxisSweep3(worldAabbMin, worldAabbMax);
    break;
   default:
    assert (false);
  }
 }

 private GLDrawElements shape_to_mesh(btCollisionShape collisionShape) {
  if (collisionShape.isConvex()) {
   btConvexShape convex = (btConvexShape) collisionShape;
   btShapeHull hull = new btShapeHull(convex);
   hull.buildHull(0.0f);
   ArrayIntList indices = new ArrayIntList(hull.numTriangles() * 3);
   ArrayFloatList vertices = new ArrayFloatList(hull.numTriangles() * 3 * 6);
   int vcount = 0;
   final btVector3 triNormal = new btVector3();
   for (int t = 0; t < hull.numTriangles(); t++) {
    int index0 = hull.getIndexPointer()[t * 3 + 0];
    int index1 = hull.getIndexPointer()[t * 3 + 1];
    int index2 = hull.getIndexPointer()[t * 3 + 2];
    final btVector3 pos0 = new btVector3(hull.getVertexPointer()[index0]);
    final btVector3 pos1 = new btVector3(hull.getVertexPointer()[index1]);
    final btVector3 pos2 = new btVector3(hull.getVertexPointer()[index2]);
    triNormal.set(new btVector3(pos1).sub(pos0).cross(new btVector3(pos2).sub(pos0)));
    triNormal.normalize();
    for (int v = 0; v < 3; v++) {
     int index = hull.getIndexPointer()[t * 3 + v];
     final btVector3 pos = new btVector3(hull.getVertexPointer()[index]);
     vertices.add(pos.x);
     vertices.add(pos.y);
     vertices.add(pos.z);
     vertices.add(triNormal.x);
     vertices.add(triNormal.y);
     vertices.add(triNormal.z);
     indices.add(vcount);
     ++vcount;
    }
   }
   return new TriangleElement(vertices.toBackedArray(), indices.toBackedArray());
  } else if (collisionShape.isConcave()) {
   btTriangleMeshShape mesh = (btTriangleMeshShape) collisionShape;
   List<btVector3> triangles = new ArrayList<>();
   mesh.processAllTriangles(
    (vertex, part, id) -> {
    triangles.add(new btVector3(vertex[0]));
    triangles.add(new btVector3(vertex[1]));
    triangles.add(new btVector3(vertex[2]));
    return true;
   },
    new btVector3(-FLT_MAX, -FLT_MAX, -FLT_MAX), new btVector3(FLT_MAX, FLT_MAX, FLT_MAX));
   /*
    * remove duplicate vertices
    *
    */
   List<btVector3> vertices = triangles.stream()
    .collect(Collectors.toSet())
    .stream()
    .collect(Collectors.toList());
   ArrayIntList indices;
   {
    /*
     * map triangle vertices to the new index into the deduplicated vertex list
     *
     */
    Map<btVector3, Integer> vertex_lookup = IntStream
     .range(0, vertices.size())
     .mapToObj(o -> new SimpleEntry<>(vertices.get(o), o))
     .collect(Collectors.toMap(SimpleEntry::getKey, SimpleEntry::getValue));
    /*
     * Create triangle mesh indices
     *
     */
    indices = triangles.stream()
     .map(o -> vertex_lookup.get(o))
     .collect(ArrayIntList::new, ArrayIntList::add, ArrayIntList::addAll);
   }
   /*
     * calculate face normals }
    */
   Map<Integer, btVector3> normal_lookup;
   {
    List<btVector3> face_normals =
     IntStream.range(0, indices.size() / 3)
     .mapToObj(o -> new int[]{indices.get(o * 3), indices.get(o * 3 + 1), indices.get(o * 3 + 2)})
     .map(o -> new btVector3[]{vertices.get(o[0]), vertices.get(o[1]), vertices.get(o[2])})
     .map(o -> new btVector3(o[1]).sub(o[0]).cross(new btVector3(o[2]).sub(o[0])))
     .peek(o -> o.normalize())
     .collect(Collectors.toList());
    /*
      * Find vertex normals by average the shared face normals
      *
     */
    normal_lookup = IntStream.range(0, indices.size())
     .parallel()
     .mapToObj(o -> new SimpleEntry<Integer, btVector3>(indices.get(o), face_normals.get(o / 3)))
     .collect(
      Collectors.groupingByConcurrent(
       SimpleEntry::getKey,
       mapping(SimpleEntry::getValue, toList())))
     .entrySet()
     .parallelStream()
     .map(o -> new SimpleEntry<Integer, btVector3>(
      o.getKey(),
      o.getValue().stream()
      .reduce(new btVector3(),
       (a, b) -> a.add(b),
       (a, b) -> a.add(b))
      .scale(1.0f / o.getValue().size())
      .normalize()))
     .collect(toMap(SimpleEntry::getKey, SimpleEntry::getValue));
   }
   assert (vertices.size() == normal_lookup.size());
   ArrayFloatList vaa = new ArrayFloatList((vertices.size() + normal_lookup.size()) * 3);
   int iv = 0;
   for (btVector3 v : vertices) {
    vaa.add(v.x);
    vaa.add(v.y);
    vaa.add(v.z);
    final btVector3 n = normal_lookup.get(iv);
    vaa.add(n.x);
    vaa.add(n.y);
    vaa.add(n.z);
    ++iv;
   }
   GLDrawElements shape = new TriangleElement(vaa.toBackedArray(), indices.toBackedArray());
   return shape;
  }
  return null;
 }

 public void create_ground() {
  ///create a few basic rigid bodies
  btCollisionShape groundShape = new btBoxShape(new btVector3(250.0F, 50.0F, 250.0F));
  //	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),0);
  final btTransform groundTransform = new btTransform();
  groundTransform.setIdentity();
  groundTransform.setOrigin(new btVector3(0.0F, -50.0F, 0.0F));
  //We can also use DemoApplication::createRigidBody, but for clarity it is provided here:
  {
   float mass = 0.0F;
   //rigidbody is dynamic if and only if mass is non zero, otherwise static
   boolean isDynamic = mass != 0.0F;
   final btVector3 localInertia = new btVector3();
   if (isDynamic) {
    groundShape.calculateLocalInertia(mass, localInertia);
   }
   //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
   btDefaultMotionState myMotionState = new btDefaultMotionState(groundTransform);
   btRigidBodyConstructionInfo rbInfo =
    new btRigidBodyConstructionInfo(mass, myMotionState, groundShape, localInertia);
   btRigidBody body = new btRigidBody(rbInfo);
   //add the body to the dynamics world
   world().addRigidBody(body);
  }
 }
ArrayFloatList[] LandscapeVtx = {
  Landscape01Vtx,
  Landscape02Vtx,
  Landscape03Vtx,
  Landscape04Vtx,
  Landscape05Vtx,
  Landscape06Vtx,
  Landscape07Vtx,
  Landscape08Vtx,};
 int[] LandscapeVtxCount = {
  Landscape01Vtx.size() / 3,
  Landscape02Vtx.size() / 3,
  Landscape03Vtx.size() / 3,
  Landscape04Vtx.size() / 3,
  Landscape05Vtx.size() / 3,
  Landscape06Vtx.size() / 3,
  Landscape07Vtx.size() / 3,
  Landscape08Vtx.size() / 3,};
 ArrayIntList[] LandscapeIdx = {
  Landscape01Idx,
  Landscape02Idx,
  Landscape03Idx,
  Landscape04Idx,
  Landscape05Idx,
  Landscape06Idx,
  Landscape07Idx,
  Landscape08Idx,};
 int[] LandscapeIdxCount = {
  Landscape01Idx.size(),
  Landscape02Idx.size(),
  Landscape03Idx.size(),
  Landscape04Idx.size(),
  Landscape05Idx.size(),
  Landscape06Idx.size(),
  Landscape07Idx.size(),
  Landscape08Idx.size(),};
 protected void createLargeMeshBody() {
  createLargeMeshBody(new btVector3(1,1,1));
 }

 protected void createLargeMeshBody(final btVector3 scaling) {
  final btTransform trans = new btTransform();
  trans.setIdentity();
  for (int i = 0; i < 8;
   i++) {
   btTriangleIndexVertexArray meshInterface = new btTriangleIndexVertexArray();
   btIndexedMesh part = new btIndexedMesh();
   part.m_vertexBase = LandscapeVtx[i];
   part.m_vertexStride = 3;
   part.m_numVertices = LandscapeVtxCount[i];
   part.m_triangleIndexBase = LandscapeIdx[i];
   part.m_triangleIndexStride = 1;
   part.m_numTriangles = LandscapeIdxCount[i] / 3;
   meshInterface.addIndexedMesh(part);
   boolean useQuantizedAabbCompression = true;
   btBvhTriangleMeshShape trimeshShape =
    new btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression);
   trimeshShape.setLocalScaling(scaling);
   final btVector3 localInertia = new btVector3();
   trans.setOrigin(new btVector3(0, -25, 0));
   btRigidBody body = createRigidBody(0, trans, trimeshShape);
   body.setFriction(0.9F);
  }
 }
}
