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

/**
 *
 * @author Gregery Barton
 */
public class Throttle {

 private final int tps;
 private final int max_updates;
 private int updates_allowed;
 private long last_update_nano = System.nanoTime();

 public Throttle(int tps, int max_updates) {
  this.tps = tps;
  this.max_updates = max_updates;
 }

 public boolean update_now() {
  // Throttle GUI updates to avoid non-responsiveness
  if (updates_allowed > 0) {
   --updates_allowed;
   return true;
  } else {
   long now = System.nanoTime();
   if ((now - last_update_nano) > (1_000_000_000l / tps)) {
    last_update_nano = now;
    updates_allowed = max_updates - 1;
    return true;
   } else {
    return false;
   }
  }
 }
}
