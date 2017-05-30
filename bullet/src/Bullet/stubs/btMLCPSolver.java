/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
///original version written by Erwin Coumans, October 2013
package Bullet.stubs;

import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btPersistentManifold;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.ConstraintSolver.btConstraintSolver;
import Bullet.Dynamics.btContactSolverInfo;
import java.io.Serializable;
import java.util.List;

/**
 *
 * @author Gregery Barton
 */
public class btMLCPSolver extends btConstraintSolver implements Serializable {

 public btMLCPSolver(btDantzigSolver mlcp) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 @Override
 public float solveGroup(List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifold, int numManifolds, List<btTypedConstraint> constraints,
  int numConstraints, btContactSolverInfo info, btIDebugDraw debugDrawer, btDispatcher dispatcher) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 @Override
 public void reset() {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 @Override
 public int getSolverType() {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 public int getNumFallbacks() {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }

 public void setNumFallbacks(int i) {
  throw new UnsupportedOperationException("Not supported yet."); //To change body of generated methods, choose Tools | Templates.
 }
}
