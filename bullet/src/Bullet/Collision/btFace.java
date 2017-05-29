/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */
package Bullet.Collision;

import java.io.Serializable;
import java.util.Arrays;

/**
 *
 * @author Gregery Barton
 */
public class btFace implements Serializable {

 public int[] m_indices = new int[0];
 public final float[] m_plane = new float[4];

 public btFace() {
 }

 public btFace(btFace o) {
  m_indices = Arrays.copyOf(o.m_indices, o.m_indices.length);
  System.arraycopy(o.m_plane, 0, m_plane, 0, 4);
 }
};
