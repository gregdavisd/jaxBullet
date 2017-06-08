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
package bullet_examples.apps.benchmarks;

import Bullet.Collision.Algorithm.btCollisionDispatcher;
import Bullet.Collision.btDefaultCollisionConfiguration;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_SIMD;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_USE_WARMSTARTING;
import Bullet.Dynamics.ConstraintSolver.btSequentialImpulseConstraintSolver;
import Bullet.Dynamics.btDiscreteDynamicsWorld;
import Bullet.LinearMath.btQuaternion;
import Bullet.LinearMath.btVector3;
import bullet_examples.DebugDraw;
import bullet_examples.DemoContainer;

/**
 *
 * @author Gregery Barton
 */
public abstract class BenchmarkDemoContainer extends DemoContainer {

 protected static final float COLLISION_RADIUS = 0.0f;
 static int gNumIslands = 0;
 static int gSolverMode = SOLVER_SIMD |
  SOLVER_USE_WARMSTARTING // SOLVER_RANDMIZE_ORDER |
  // SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS |
  // SOLVER_USE_2_FRICTION_DIRECTIONS |
  ;

 @Override
 protected void initWorld(String broadphase_class) {
  gNumIslands = 0;
  collision_configuration = new btDefaultCollisionConfiguration();
  ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
  dispatcher = new btCollisionDispatcher(collision_configuration);
  create_broadphase(broadphase_class);
  constraint_solver = new btSequentialImpulseConstraintSolver();
  world = new btDiscreteDynamicsWorld(dispatcher, broadphase, constraint_solver,
   collision_configuration);
  world.setGravity(new btVector3(0, -10, 0));
  world.getSolverInfo().m_solverMode = gSolverMode;
  debug_draw = new DebugDraw();
  world.setDebugDrawer(debug_draw);
  ///the following 3 lines increase the performance dramatically, with a little bit of loss of quality
  world.getSolverInfo().m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING; //don't recalculate friction values each frame
  world.getSolverInfo().m_numIterations = 5; //few solver iterations 
  //m_defaultContactProcessingThreshold = 0.f;//used when creating bodies: body->setContactProcessingThreshold(...);
 }


 @Override
 public boolean step_physics(float cap_frametime, float rate) {
  return step_physics(cap_frametime, rate, 1);
 }

 protected btVector3 rotate(final btQuaternion quat, final btVector3 vec) {
  float tmpX, tmpY, tmpZ, tmpW;
  tmpX = (((quat.getW() * vec.getX()) + (quat.getY() * vec.getZ())) - (quat.getZ() * vec.getY()));
  tmpY = (((quat.getW() * vec.getY()) + (quat.getZ() * vec.getX())) - (quat.getX() * vec.getZ()));
  tmpZ = (((quat.getW() * vec.getZ()) + (quat.getX() * vec.getY())) - (quat.getY() * vec.getX()));
  tmpW = (((quat.getX() * vec.getX()) + (quat.getY() * vec.getY())) + (quat.getZ() * vec.getZ()));
  return new btVector3(
   ((((tmpW * quat.getX()) + (tmpX * quat.getW())) - (tmpY * quat.getZ())) + (tmpZ * quat.getY())),
   ((((tmpW * quat.getY()) + (tmpY * quat.getW())) - (tmpZ * quat.getX())) + (tmpX * quat.getZ())),
   ((((tmpW * quat.getZ()) + (tmpZ * quat.getW())) - (tmpX * quat.getY())) + (tmpY * quat.getX()))
  );
 }
 
}
