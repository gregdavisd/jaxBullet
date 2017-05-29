/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics;

import Bullet.stubs.btStackAlloc;
import Bullet.Collision.Broadphase.btDispatcher;
import Bullet.Collision.btCollisionObject;
import Bullet.Collision.btIDebugDraw;
import Bullet.Collision.btPersistentManifold;
import Bullet.Collision.btSimulationIslandManager;
import Bullet.Dynamics.Constraint.btTypedConstraint;
import Bullet.Dynamics.ConstraintSolver.btConstraintSolver;
import static Bullet.Dynamics.btDiscreteDynamicsWorld.btGetConstraintIslandId;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author Gregery Barton
 */
public class InplaceSolverIslandCallback extends btSimulationIslandManager.IslandCallback implements
 Serializable {

 btContactSolverInfo m_solverInfo;
 btConstraintSolver m_solver;
 ArrayList<btTypedConstraint> m_sortedConstraints;
 int m_numConstraints;
 btIDebugDraw m_debugDrawer;
 btDispatcher m_dispatcher;
 final ArrayList<btCollisionObject> m_bodies = new ArrayList<>(0);
 final ArrayList<btPersistentManifold> m_manifolds = new ArrayList<>(0);
 final ArrayList<btTypedConstraint> m_constraints = new ArrayList<>(0);

 InplaceSolverIslandCallback(
  btConstraintSolver solver,
  btStackAlloc stackAlloc,
  btDispatcher dispatcher) {
  m_solverInfo = null;
  m_solver = solver;
  m_sortedConstraints = null;
  m_numConstraints = 0;
  m_debugDrawer = null;
  m_dispatcher = dispatcher;
 }

 void setup(btContactSolverInfo solverInfo, ArrayList<btTypedConstraint> sortedConstraints,
  int numConstraints, btIDebugDraw debugDrawer) {
  assert (solverInfo != null);
  m_solverInfo = solverInfo;
  m_sortedConstraints = sortedConstraints;
  m_numConstraints = numConstraints;
  m_debugDrawer = debugDrawer;
  m_bodies.clear();
  m_manifolds.clear();
  m_constraints.clear();
 }

 @Override
 public void processIsland(List<btCollisionObject> bodies, int numBodies,
  List<btPersistentManifold> manifolds, int numManifolds, int islandId) {
  if (islandId < 0) {
   ///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
   m_solver.solveGroup(bodies, numBodies, manifolds, numManifolds, m_sortedConstraints,
    m_numConstraints, m_solverInfo, m_debugDrawer, m_dispatcher);
  } else {
   //also add all non-contact constraints/joints for this island
   int startConstraint = 0;
   int numCurConstraints = 0;
   int i;
   //find the first constraint for this island
   for (i = 0; i < m_numConstraints; i++) {
    if (btGetConstraintIslandId(m_sortedConstraints.get(i)) == islandId) {
     startConstraint = i;
     break;
    }
   }
   //count the number of constraints in this island
   for (; i < m_numConstraints; i++) {
    if (btGetConstraintIslandId(m_sortedConstraints.get(i)) == islandId) {
     numCurConstraints++;
    }
   }
   if (m_solverInfo.m_minimumSolverBatchSize <= 1) {
    m_solver.solveGroup(bodies, numBodies, manifolds, numManifolds, (ArrayList) m_sortedConstraints
     .subList(
      startConstraint, startConstraint + numCurConstraints), numCurConstraints, m_solverInfo,
     m_debugDrawer, m_dispatcher);
   } else {
    for (i = 0; i < numBodies; i++) {
     m_bodies.add(bodies.get(i));
    }
    for (i = 0; i < numManifolds; i++) {
     m_manifolds.add(manifolds.get(i));
    }
    for (i = 0; i < numCurConstraints; i++) {
     m_constraints.add(m_sortedConstraints.get(startConstraint + i));
    }
    if ((m_constraints.size() + m_manifolds.size()) > m_solverInfo.m_minimumSolverBatchSize) {
     processConstraints();
    } else {
     //printf("deferred\n");
    }
   }
  }
 }

 void processConstraints() {
//		ArrayList<btCollisionObject> bodies = !m_bodies.isEmpty()? m_bodies:null;
//		ArrayList<btPersistentManifold> manifold = !m_manifolds.isEmpty()?m_manifolds:null;
//		ArrayList<btTypedConstraint> constraints = !m_constraints.isEmpty()?m_constraints:null;
  m_solver.solveGroup(m_bodies, m_bodies.size(), m_manifolds, m_manifolds.size(), m_constraints,
   m_constraints.size(), m_solverInfo, m_debugDrawer, m_dispatcher);
  m_bodies.clear();
  m_manifolds.clear();
  m_constraints.clear();
 }
};
