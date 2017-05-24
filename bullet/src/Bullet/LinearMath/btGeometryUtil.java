/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

package Bullet.LinearMath;

import static Bullet.LinearMath.btScalar.btFabs;
import java.io.Serializable;
import java.util.ArrayList;

/**
 *
 * @author Gregery Barton
 */
public class btGeometryUtil  implements Serializable {

 /**
  *
  * @param vertices
  * @param planeEquationsOut
  */
 public static void getPlaneEquationsFromVertices(ArrayList<btVector3> vertices,
  ArrayList<btVector4> planeEquationsOut) {
  final int numvertices = vertices.size();
  // brute force:
  final btVector3 N3 = new btVector3();
  final btVector4 planeEquation = new btVector4();
  final btVector3 planeEquation3 = new btVector3();
  final btVector3 edge0 = new btVector3();
  final btVector3 edge1 = new btVector3();
  for (int i = 0; i < numvertices; i++) {
   final btVector3 N1 = vertices.get(i);
   for (int j = i + 1; j < numvertices; j++) {
    final btVector3 N2 = vertices.get(j);
    for (int k = j + 1; k < numvertices; k++) {
     N3.set(vertices.get(k));
     edge0.set(N2).sub(N1);
     edge1.set(N3).sub(N1);
     float normalSign = 1.f;
     for (int ww = 0; ww < 2; ww++) {
      planeEquation3.set(edge0).cross(edge1).scale(normalSign);
      float l2 = planeEquation3.lengthSquared();
      if (l2 > (0.0001f)) {
       planeEquation3.scale(1.0f / l2);
       planeEquation.set(planeEquation3, 0);
       if (notExist(planeEquation, planeEquationsOut)) {
        planeEquation.w = -planeEquation3.dot(N1);
        //check if inside, and replace supportingVertexOut if needed
        if (areVerticesBehindPlane(planeEquation, vertices, (0.01f))) {
         planeEquationsOut.add(new btVector4(planeEquation));
        }
       }
      }
      normalSign = (-1.f);
     }
    }
   }
  }
 }

 /**
  *
  * @param planeEquations
  * @param verticesOut
  */
 public static void getVerticesFromPlaneEquations(ArrayList<btVector4> planeEquations,
  ArrayList<btVector3> verticesOut) {
  int numbrushes = planeEquations.size();
  // brute force:
  final btVector3 tN1 = new btVector3();
  final btVector3 tN2 = new btVector3();
  final btVector3 tN3 = new btVector3();
  final btVector3 n2n3 = new btVector3();
  final btVector3 n3n1 = new btVector3();
  final btVector3 n1n2 = new btVector3();
  final btVector3 potentialVertex = new btVector3();
  for (int i = 0; i < numbrushes; i++) {
   final btVector4 N1 = planeEquations.get(i);
   for (int j = i + 1; j < numbrushes; j++) {
    final btVector4 N2 = planeEquations.get(j);
    for (int k = j + 1; k < numbrushes; k++) {
     final btVector4 N3 = planeEquations.get(k);
     tN1.set(N1.x, N1.y, N1.z);
     tN2.set(N2.x, N2.y, N2.z);
     tN3.set(N3.x, N3.y, N3.z);
     n2n3.set(tN2).cross(tN3);
     n3n1.set(tN3).cross(tN1);
     n1n2.set(tN1).cross(tN2);
     if ((n2n3.lengthSquared() > (0.0001f)) &&
      (n3n1.lengthSquared() > (0.0001f)) &&
      (n1n2.lengthSquared() > (0.0001f))) {
      //point P out of 3 plane equations:
      //	d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )  
      //P =  -------------------------------------------------------------------------  
      //   N1 . ( N2 * N3 )  
      float quotient = (tN1.dot(n2n3));
      if (btFabs(quotient) > (0.000001f)) {
       quotient = (-1.f) / quotient;
       n2n3.scale(N1.w);
       n3n1.scale(N2.w);
       n1n2.scale(N3.w);
       potentialVertex.set(n2n3);
       potentialVertex.add(n3n1);
       potentialVertex.add(n1n2);
       potentialVertex.scale(quotient);
       //check if inside, and replace supportingVertexOut if needed
       if (isPointInsidePlanes(planeEquations, potentialVertex, (0.01f))) {
        verticesOut.add(new btVector3(potentialVertex));
       }
      }
     }
    }
   }
  }
 }

 /**
  *
  * @param vertices
  * @param planeNormal
  * @param margin
  * @return
  */
 public static boolean isInside(ArrayList<btVector3> vertices, final btVector3 planeNormal,
  float margin) {
  // can't find c++ definition
  throw new AssertionError();
 }

 /**
  *
  * @param planeEquations
  * @param point
  * @param margin
  * @return
  */
 public static boolean isPointInsidePlanes(ArrayList<btVector4> planeEquations,
  final btVector3 point,
  float margin) {
  int numbrushes = planeEquations.size();
  btVector4 point4 = new btVector4(point, 0f);
  for (int i = 0; i < numbrushes; i++) {
   final btVector4 N1 = planeEquations.get(i);
   float dist = (N1.dot(point4)) + (N1.w) - margin;
   if (dist > (0.f)) {
    return false;
   }
  }
  return true;
 }

 /**
  *
  * @param planeNormal
  * @param vertices
  * @param margin
  * @return
  */
 public static boolean areVerticesBehindPlane(final btVector4 planeNormal,
  ArrayList<btVector3> vertices,
  float margin) {
  int numvertices = vertices.size();
  btVector4 N1_4 = new btVector4();
  for (int i = 0; i < numvertices; i++) {
   final btVector3 N1 = vertices.get(i);
   float dist = (planeNormal.dot(N1_4.set(N1, 0f))) + (planeNormal.w) - margin;
   if (dist > (0.f)) {
    return false;
   }
  }
  return true;
 }

 /**
  *
  * @param planeEquation
  * @param planeEquations
  * @return
  */
 public static boolean notExist(final btVector4 planeEquation, ArrayList<btVector4> planeEquations) {
  int numbrushes = planeEquations.size();
  for (int i = 0; i < numbrushes; i++) {
   final btVector4 N1 = planeEquations.get(i);
   if (planeEquation.dot(N1) > (0.999f)) {
    return false;
   }
  }
  return true;
 }
}
