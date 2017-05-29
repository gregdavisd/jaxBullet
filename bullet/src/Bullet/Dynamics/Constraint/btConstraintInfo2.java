/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2010 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Dynamics.Constraint;

import Bullet.LinearMath.btVector3;
import java.io.Serializable;
import javax.vecmath.FloatSmartPointer;

/**
 *
 * @author Gregery Barton
 */
public class btConstraintInfo2 implements Serializable {

 // integrator parameters: frames per second (1/stepsize), default error
 // reduction parameter (0..1).
 public float fps, erp;
 // for the first and second body, pointers to two (linear and angular)
 // n*3 jacobian sub matrices, stored by rows. these matrices will have
 // been initialized to 0 on entry. if the second body is zero then the
 // J2xx pointers may be 0.
 //float *m_J1linearAxis,*m_J1angularAxis,*m_J2linearAxis,*m_J2angularAxis;
 public btVector3[] m_J1linearAxis;
 public btVector3[] m_J1angularAxis;
 public btVector3[] m_J2linearAxis;
 public btVector3[] m_J2angularAxis;
 // elements to jump from one row to the next in J's
 public final int rowskip = 1;
 // right hand sides of the equation J*v = c + cfm * lambda. cfm is the
 // "constraint force mixing" vector. c is set to zero on entry, cfm is
 // set to a constant value (typically very small or zero) value on entry.
 public FloatSmartPointer[] m_constraintError;
 public FloatSmartPointer[] cfm;
 // lo and hi limits for variables (set to -/+ infinity on entry).
 public FloatSmartPointer[] m_lowerLimit;
 public FloatSmartPointer[] m_upperLimit;
 // number of solver iterations
 public int m_numIterations;
 //damping of the velocity
 public float m_damping;
}
