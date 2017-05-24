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
package Bullet.Dynamics.ConstraintSolver;

import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btPersistentManifold;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.btContactSolverInfo;
import java.io.Serializable;
import java.util.List;

/**
 *
 * @author Gregery Barton
 */
public abstract class btConstraintSolver  implements Serializable {

 public void prepareSolve(int numBodies, int numManifolds) {;
 }

 ///solve a group of constraints
 public abstract float solveGroup(List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifold, int numManifolds,
  List<btTypedConstraint> constraints, int numConstraints, btContactSolverInfo info,
  btIDebugDraw debugDrawer, btDispatcher dispatcher);

 public void allSolved(btContactSolverInfo info, btIDebugDraw debugDrawer) {
 }

 ///clear internal cached data and reset random seed
public  abstract void reset();

public  abstract int getSolverType();
}
