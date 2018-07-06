/*
 * Copyright (c) 2017  Gregery Barton
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

import Bullet.Collision.Broadphase.btAxisSweep3;
import Bullet.Collision.Broadphase.btDbvtBroadphase;
import Bullet.Collision.Algorithm.btCollisionDispatcher;
import Bullet.Collision.btDefaultCollisionConfiguration;
import Bullet.Dynamics.ConstraintSolver.btSequentialImpulseConstraintSolver;
import Bullet.Dynamics.btDiscreteDynamicsWorld;
import Bullet.LinearMath.btVector3;

/**
 *
 * @author Gregery Barton
 */
abstract public class DiscreteDemoContainer extends DemoContainer {

 @Override
 protected void initWorld(String broadphase_class) {
  collision_configuration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collision_configuration);
  create_broadphase(broadphase_class);
  constraint_solver = new btSequentialImpulseConstraintSolver();
  world = new btDiscreteDynamicsWorld(dispatcher, broadphase, constraint_solver,
   collision_configuration);
  world.setGravity(new btVector3(0, -10, 0));
  debug_draw = new DebugDraw();
  world.setDebugDrawer(debug_draw);
 }

}
