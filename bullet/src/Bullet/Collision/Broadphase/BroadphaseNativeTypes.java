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
package Bullet.Collision.Broadphase;

import java.io.Serializable;

/**
 *
 * @author Gregery Barton
 */
public class BroadphaseNativeTypes implements Serializable  {

 // polyhedral convex shapes
 public static final int BOX_SHAPE_PROXYTYPE = 0;
 public static final int TRIANGLE_SHAPE_PROXYTYPE = 1;
 public static final int TETRAHEDRAL_SHAPE_PROXYTYPE = 2;
 public static final int CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE = 3;
 public static final int CONVEX_HULL_SHAPE_PROXYTYPE = 4;
 public static final int CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE = 5;
 public static final int CUSTOM_POLYHEDRAL_SHAPE_TYPE = 6;
//implicit convex shapes
 public static final int IMPLICIT_CONVEX_SHAPES_START_HERE = 7;
 public static final int SPHERE_SHAPE_PROXYTYPE = 8;
 public static final int MULTI_SPHERE_SHAPE_PROXYTYPE = 9;
 public static final int CAPSULE_SHAPE_PROXYTYPE = 10;
 public static final int CONE_SHAPE_PROXYTYPE = 11;
 public static final int CONVEX_SHAPE_PROXYTYPE = 12;
 public static final int CYLINDER_SHAPE_PROXYTYPE = 13;
 public static final int UNIFORM_SCALING_SHAPE_PROXYTYPE = 14;
 public static final int MINKOWSKI_SUM_SHAPE_PROXYTYPE = 15;
 public static final int MINKOWSKI_DIFFERENCE_SHAPE_PROXYTYPE = 16;
 public static final int BOX_2D_SHAPE_PROXYTYPE = 17;
 public static final int CONVEX_2D_SHAPE_PROXYTYPE = 18;
 public static final int CUSTOM_CONVEX_SHAPE_TYPE = 19;
//concave shapes
 public static final int CONCAVE_SHAPES_START_HERE = 20;
 //keep all the convex shapetype below here, for the check IsConvexShape in broadphase proxy!
 public static final int TRIANGLE_MESH_SHAPE_PROXYTYPE = 21;
 public static final int SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE = 22;
 ///used for demo integration FAST/Swift collision library and Bullet
 public static final int FAST_CONCAVE_MESH_PROXYTYPE = 23;
 //terrain
 public static final int TERRAIN_SHAPE_PROXYTYPE = 24;
///Used for GIMPACT Trimesh integration
 public static final int GIMPACT_SHAPE_PROXYTYPE = 25;
///Multimaterial mesh
 public static final int MULTIMATERIAL_TRIANGLE_MESH_PROXYTYPE = 26;
 public static final int EMPTY_SHAPE_PROXYTYPE = 27;
 public static final int STATIC_PLANE_PROXYTYPE = 28;
 public static final int CUSTOM_CONCAVE_SHAPE_TYPE = 29;
 public static final int CONCAVE_SHAPES_END_HERE = 30;
 public static final int COMPOUND_SHAPE_PROXYTYPE = 31;
 public static final int SOFTBODY_SHAPE_PROXYTYPE = 32;
 public static final int HFFLUID_SHAPE_PROXYTYPE = 33;
 public static final int HFFLUID_BUOYANT_CONVEX_SHAPE_PROXYTYPE = 34;
 public static final int INVALID_SHAPE_PROXYTYPE = 35;
 public static final int MAX_BROADPHASE_COLLISION_TYPES = 36;
}
