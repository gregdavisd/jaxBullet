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

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Arrays;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.apache.commons.collections.primitives.ArrayFloatList;
import org.apache.commons.collections.primitives.ArrayIntList;

/**
 *
 * @author Gregery Barton
 */
public class LandscapeData {

 public static  ArrayFloatList Landscape01Vtx ;
 public static  ArrayFloatList Landscape02Vtx ;
 public static  ArrayFloatList Landscape03Vtx ;
 public static  ArrayFloatList Landscape04Vtx ;
 public static  ArrayFloatList Landscape05Vtx ;
 public static  ArrayFloatList Landscape06Vtx ;
 public static  ArrayFloatList Landscape07Vtx ;
 public static  ArrayFloatList Landscape08Vtx ;
 public static  ArrayIntList Landscape01Idx ;
 public static  ArrayIntList Landscape02Idx ;
 public static  ArrayIntList Landscape03Idx ;
 public static  ArrayIntList Landscape04Idx ;
 public static  ArrayIntList Landscape05Idx ;
 public static  ArrayIntList Landscape06Idx ;
 public static  ArrayIntList Landscape07Idx ;
 public static  ArrayIntList Landscape08Idx ;

 private static ArrayFloatList string_to_floats(String s) {
  String[] split = s.replace('\n', ' ').split(",");
  return Arrays.stream(split)
   .map(o -> o.trim())
   .mapToDouble(o -> Double.parseDouble(o))
   .collect(ArrayFloatList::new, ArrayFloatList::add, ArrayFloatList::addAll);
 }

 private static ArrayIntList string_to_ints(String s) {
  String[] split = s.replace('\n', ' ').split(",");
  return Arrays.stream(split)
   .map(o -> o.trim())
   .mapToInt(o -> Integer.parseInt(o))
   .collect(ArrayIntList::new, ArrayIntList::add, ArrayIntList::addAll);
 }

 static {
  try (InputStream in = LandscapeData.class.getResource("LandscapeData.xml").openStream()) {
   Properties p = new Properties();
   p.loadFromXML(in);
   Landscape01Vtx =string_to_floats(p.getProperty("Landscape01Vtx"));
   Landscape02Vtx = string_to_floats(p.getProperty("Landscape02Vtx"));
   Landscape03Vtx = string_to_floats(p.getProperty("Landscape03Vtx"));
   Landscape04Vtx = string_to_floats(p.getProperty("Landscape04Vtx"));
   Landscape05Vtx = string_to_floats(p.getProperty("Landscape05Vtx"));
   Landscape06Vtx = string_to_floats(p.getProperty("Landscape06Vtx"));
   Landscape07Vtx = string_to_floats(p.getProperty("Landscape07Vtx"));
   Landscape08Vtx = string_to_floats(p.getProperty("Landscape08Vtx"));
   Landscape01Idx = string_to_ints(p.getProperty("Landscape01Idx"));
   Landscape02Idx = string_to_ints(p.getProperty("Landscape02Idx"));
   Landscape03Idx = string_to_ints(p.getProperty("Landscape03Idx"));
   Landscape04Idx = string_to_ints(p.getProperty("Landscape04Idx"));
   Landscape05Idx = string_to_ints(p.getProperty("Landscape05Idx"));
   Landscape06Idx = string_to_ints(p.getProperty("Landscape06Idx"));
   Landscape07Idx = string_to_ints(p.getProperty("Landscape07Idx"));
   Landscape08Idx = string_to_ints(p.getProperty("Landscape08Idx"));
  } catch (FileNotFoundException ex) {
   Logger.getLogger(LandscapeData.class.getName()).log(Level.SEVERE, null, ex);
  } catch (IOException ex) {
   Logger.getLogger(LandscapeData.class.getName()).log(Level.SEVERE, null, ex);
  }
 }
}
