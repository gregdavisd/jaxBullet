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

import java.awt.Canvas;
import java.awt.Component;
import java.awt.Container;
import java.awt.Cursor;
import java.awt.Insets;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.event.MouseEvent;
import static java.awt.event.MouseEvent.BUTTON1;
import static java.awt.event.MouseEvent.BUTTON2;
import static java.awt.event.MouseEvent.BUTTON3;
import static java.awt.event.MouseEvent.MOUSE_MOVED;
import static java.awt.event.MouseEvent.MOUSE_PRESSED;
import static java.awt.event.MouseEvent.MOUSE_RELEASED;
import java.awt.event.MouseListener;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import javax.swing.SwingUtilities;
import javax.vecmath.Point2i;
import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;

/**
 *
 * @author Gregery Barton
 */
public class OpenGLFrame extends javax.swing.JFrame implements MouseListener,
 MouseMotionListener, MouseWheelListener {

 private static final long serialVersionUID = 3281858039089406479L;
 private final Cursor blank_cursor;
 private org.lwjgl.input.Cursor lwjgl_blank_cursor;
 private final Point2i grabbed_pos = new Point2i();

 /**
  * Creates new form BulletOpenGLFrame
  */
 public OpenGLFrame() {
  initComponents();
  blank_cursor = Toolkit.getDefaultToolkit().createCustomCursor(Toolkit
   .getDefaultToolkit()
   .createImage(new byte[0]), new Point(), "blank");
 }

 private final ArrayBlockingQueue<Runnable> queue = new ArrayBlockingQueue<>(16);
 private MouseListener mouse;
 private MouseMotionListener mouse_motion;
 private MouseWheelListener mouse_wheel;

 static class LWJGLMouseEvent {

  int getEventButton;
  boolean getEventButtonState;
  int getEventDWheel;
  int getEventDX;
  int getEventDY;
  long getEventNanoseconds;
  int getEventX;
  int getEventY;

  public LWJGLMouseEvent(int getEventButton, boolean getEventButtonState,
   int getEventDWheel,
   int getEventDX, int getEventDY, long getEventNanoseconds, int getEventX,
   int getEventY) {
   this.getEventButton = getEventButton;
   this.getEventButtonState = getEventButtonState;
   this.getEventDWheel = getEventDWheel;
   this.getEventDX = getEventDX;
   this.getEventDY = getEventDY;
   this.getEventNanoseconds = getEventNanoseconds;
   this.getEventX = getEventX;
   this.getEventY = getEventY;
  }

 }

 private void get_lwjgl_mouse_events(List<LWJGLMouseEvent> events) {
  /*
   * turn polled LWJGL events into a list
   */
  if (Mouse.isCreated()) {
   while (Mouse.next()) {
    events.add(new LWJGLMouseEvent(Mouse.getEventButton(), Mouse
     .getEventButtonState(), Mouse
      .getEventDWheel(), Mouse.getDX(), Mouse.getDY(),
     Mouse.getEventNanoseconds(), Mouse.getEventX(), Mouse.getEventY()));
   }
  }
 }

 public void dispatch_events() {
  final List<LWJGLMouseEvent> mouse_events = new ArrayList<>(0);
  get_lwjgl_mouse_events(mouse_events);
  queue_lwjgl_events(mouse_events);
  if (mouse != null && mouse_motion != null && mouse_wheel != null) {
   Runnable runnable;
   while ((runnable = queue.poll()) != null) {
    runnable.run();
   }
  }
 }

 private MouseEvent apply_grabbing_to_event(MouseEvent e) {
  if (!is_grabbed()) {
   return e;
  } else {
   /*
    * use grabbed_base as reference and add deltas to grabbed_pos which is a
    * large virtual mouse space. grabbed_base is the previous mouse position for
    * calculating the delta, grabbed_base is also reset by center_mouse_cursor
    */
   Point2i current = new Point2i(e.getX(), e.getY());
   Point2i delta = new Point2i(current).sub(grabbed_base);
   grabbed_pos.add(delta);
   grabbed_base.add(delta);
   if (!(e instanceof MouseWheelEvent)) {
    MouseEvent new_e = new MouseEvent((Component) e.getSource(), e.getID(), e
     .getWhen(),
     e.getModifiers(), grabbed_pos.getX(), grabbed_pos.getY(), 0, 0, e
     .getClickCount(), e
      .isPopupTrigger(),
     e.getButton());
    return new_e;
   } else {
    MouseWheelEvent we = (MouseWheelEvent) e;
    MouseWheelEvent new_e = new MouseWheelEvent((Component) e.getSource(), e
     .getID(), e.getWhen(),
     e.getModifiers(), grabbed_pos.getX(), grabbed_pos.getY(), e.getXOnScreen(),
     e.getYOnScreen(),
     e.getClickCount(),
     e.isPopupTrigger(), we.getScrollType(), we.getScrollAmount(), we
     .getWheelRotation(), we
      .getPreciseWheelRotation());
    return new_e;
   }
  }
 }

 public void update_display() {
  Display.update();
  dispatch_events();
 }

 private Point2i lwjgl_mouse_canvas_pos_to_awt_canvas(Point2i pos) {
  pos.set(pos.x, canvas.getHeight() - pos.y - 1);
  return pos;
 }

 private Point2i awt_mouse_canvas_pos_to_lwjgl_canvas(Point2i pos) {
  pos.set(pos.x, canvas.getHeight() - pos.y + 1);
  return pos;
 }

 private boolean grabbed = false;

 public boolean is_grabbed() {
  return grabbed;
 }

 final Point2i start_grabbed = new Point2i();
 final Point2i grabbed_base = new Point2i();

 public void set_grabbed(boolean grabbed) {
  if (grabbed != this.grabbed) {
   this.grabbed = grabbed;
   if (grabbed) {
//    if (lwjgl_blank_cursor == null) {
//     try {
//      Mouse.create();
//      lwjgl_blank_cursor = new org.lwjgl.input.Cursor(32, 32, 0, 0, 1, BufferUtils.createIntBuffer(
//       32 * 32),
//       BufferUtils.createIntBuffer(32 * 32));
//     } catch (LWJGLException ex) {
//     }
//    }
//    try {
//     Mouse.setNativeCursor(lwjgl_blank_cursor);
//     Mouse.updateCursor();
//    } catch (LWJGLException ex) {
//     Logger.getLogger(OpenGLFrame.class.getName()).log(Level.SEVERE, null, ex);
//    }
    setCursor(blank_cursor);
    getGlassPane().setCursor(blank_cursor);
    start_grabbed.set(get_mouse_position());
    grabbed_pos.set(start_grabbed);
    grabbed_base.set(start_grabbed);
   } else {
    awt_mouse_canvas_pos_to_lwjgl_canvas(start_grabbed);
    //Mouse.setCursorPosition(start_grabbed.x, start_grabbed.y);
    //Mouse.updateCursor();
    setCursor(null);
    getGlassPane().setCursor(null);
//    try {
//     Mouse.setNativeCursor(null);
//    } catch (LWJGLException ex) {
//     Logger.getLogger(OpenGLFrame.class.getName()).log(Level.SEVERE, null, ex);
//    }
   }
   Mouse.setGrabbed(grabbed);
  }
 }

 private Point2i get_mouse_position() {
  Mouse.updateCursor();
  Point c = getMousePosition(true);
  Point2i pt = new Point2i(Mouse.getX(), Mouse.getY());
  lwjgl_mouse_canvas_pos_to_awt_canvas(pt);
  if (c != null) {
   c = SwingUtilities.convertPoint(this, c, canvas);
   return new Point2i(c.x, c.y);
  } else {
   Point2i p = new Point2i(Mouse.getX(), Mouse.getY());
   return lwjgl_mouse_canvas_pos_to_awt_canvas(p);
  }
 }

 private void add(Runnable runnable) {
  try {
   queue.add(runnable);
  } catch (Exception ex) {
  }
 }

 private void queue_lwjgl_events(List<LWJGLMouseEvent> events) {
  /*
   * convete LWJGL events into swing events in a queue
   *
   */
  try {
   for (LWJGLMouseEvent event : events) {
    Point2i e_pos = lwjgl_mouse_canvas_pos_to_awt_canvas(new Point2i(
     event.getEventX,
     event.getEventY));
//    Point2i e_pos = new Point2i(event.getEventX, event.getEventY);
    if (event.getEventDX != 0 || event.getEventDY != 0) {
     final MouseEvent e;
     if (!grabbed) {
      e = new MouseEvent(canvas, MOUSE_MOVED, event.getEventNanoseconds, 0,
       e_pos.x, e_pos.y, 0, 0, 1, false, 0);
     } else {
      e = new MouseEvent(canvas, MOUSE_MOVED, event.getEventNanoseconds, 0,
       grabbed_pos.x + event.getEventDX, grabbed_pos.y - event.getEventDY, 0, 0,
       1, false, 0);
     }
     queue.add((Runnable) () -> mouseMoved(e));
    }
    if (event.getEventButton != -1) {
     int button;
     switch (event.getEventButton) {
      case 0:
       button = BUTTON1;
       break;
      case 1:
       button = BUTTON3;
       break;
      case 2:
       button = BUTTON2;
       break;
      default:
       button = -1;
     }
     int id;
     if (event.getEventButtonState) {
      id = MOUSE_PRESSED;
     } else {
      id = MOUSE_RELEASED;
     }
     if (button != -1) {
      final MouseEvent e
       = new MouseEvent(canvas, id, event.getEventNanoseconds, 0, e_pos.x,
        e_pos.y, 0, 0,
        1, false, button);
      queue.add(new Runnable() {
       @Override
       public void run() {
        if (e.getID() == MOUSE_PRESSED) {
         mousePressed(e);
        } else {
         mouseReleased(e);
        }
       }

      });
     }
     {
     }
    }
   }
  } catch (Exception ex) {
  }
 }

 public void set_mouse_listener(Object listener) {
  mouse = (MouseListener) listener;
  mouse_motion = (MouseMotionListener) listener;
  mouse_wheel = (MouseWheelListener) listener;
  final OpenGLFrame me = this;
  /*
   * Swing mouse events are captured and put into a queue for later processing
   * on the OpenGL thread
   *
   */
  java.awt.EventQueue.invokeLater(new Runnable() {
   @Override
   public void run() {
    //getGlassPane().setVisible(true);
    for (MouseListener l : getMouseListeners()) {
     removeMouseListener(l);
    }
//    addMouseListener(new MouseListener() {
//     @Override
//     public void mouseClicked(final MouseEvent e) {
//      add(() -> me.mouseClicked(event_glass_to_canvas(e)));
//     }
//
//     @Override
//     public void mousePressed(final MouseEvent e) {
//      add(() -> me.mousePressed(event_glass_to_canvas(e)));
//     }
//
//     @Override
//     public void mouseReleased(final MouseEvent e) {
//      add(() -> me.mouseReleased(event_glass_to_canvas(e)));
//     }
//
//     @Override
//     public void mouseEntered(final MouseEvent e) {
//      add(() ->      me.mouseEntered(e)   );
//     }
//
//     @Override
//     public void mouseExited(final MouseEvent e) {
//      add(() -> me.mouseExited(event_glass_to_canvas(e)));
//     }
//    });
    for (MouseMotionListener l : getMouseMotionListeners()) {
     removeMouseMotionListener(l);
    }
//    addMouseMotionListener(new MouseMotionListener() {
//     @Override
//     public void mouseDragged(final MouseEvent e) {
//      add(() -> me.mouseDragged(event_glass_to_canvas(e)));
//     }
//
//     @Override
//     public void mouseMoved(final MouseEvent e) {
//      //add(() -> me.mouseMoved(event_glass_to_canvas(e)));
//     }
//    });
    for (MouseWheelListener l : getMouseWheelListeners()) {
     removeMouseWheelListener(l);
    }
//    addMouseWheelListener(new MouseWheelListener() {
//     @Override
//     public void mouseWheelMoved(final MouseWheelEvent e) {
//      add(() -> me.mouseWheelMoved((MouseWheelEvent) event_glass_to_canvas(e)));
//     }
//    });
   }

  });
 }

 public void garbage_collect() {
  /*
   * remove all components that aren't the canvas
   *
   */
  garbage_collect(getRootPane());
 }

 private boolean garbage_collect(Container container) {
  boolean treasure = false;
  List<Component> garbage = new ArrayList<>(0);
  for (int i = 0; i < container.getComponentCount(); ++i) {
   Component c = container.getComponent(i);
   if (c != canvas) {
    garbage.add(c);
   } else {
    treasure = true;
   }
  }
  for (Component component : garbage) {
   if (component instanceof Container) {
    if (!garbage_collect((Container) component)) {
     treasure = true;
     continue;
    }
   }
   container.remove(component);
  }
  return !treasure;
 }

 public Canvas canvas() {
  return canvas;
 }

 /**
  * This method is called from within the constructor to initialize the form.
  * WARNING: Do NOT modify this code. The content of this method is always
  * regenerated by the Form Editor.
  */
 @SuppressWarnings("unchecked")
 // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
 private void initComponents() {

  canvas = new java.awt.Canvas();

  setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
  setTitle("OpenGL Debug Renderer");
  setPreferredSize(new java.awt.Dimension(640, 640));
  setSize(new java.awt.Dimension(640, 480));
  addComponentListener(new java.awt.event.ComponentAdapter() {
   public void componentResized(java.awt.event.ComponentEvent evt) {
    formComponentResized(evt);
   }
  });
  addWindowFocusListener(new java.awt.event.WindowFocusListener() {
   public void windowGainedFocus(java.awt.event.WindowEvent evt) {
    formWindowGainedFocus(evt);
   }
   public void windowLostFocus(java.awt.event.WindowEvent evt) {
    formWindowLostFocus(evt);
   }
  });
  getContentPane().setLayout(null);
  getContentPane().add(canvas);
  canvas.setBounds(0, 0, 640, 640);

  pack();
 }// </editor-fold>//GEN-END:initComponents

 private void formComponentResized(java.awt.event.ComponentEvent evt) {//GEN-FIRST:event_formComponentResized
  Insets insets = getInsets();
  canvas.setBounds(0, 0, getWidth() - insets.left - insets.right, getHeight()
   - insets.top - insets.bottom);
 }//GEN-LAST:event_formComponentResized

 private void formWindowLostFocus(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowLostFocus
  set_grabbed(false);
 }//GEN-LAST:event_formWindowLostFocus

 private void formWindowGainedFocus(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowGainedFocus
  // TODO add your handling code here:
 }//GEN-LAST:event_formWindowGainedFocus

 /**
  * @param args the command line arguments
  */
 public static void main(String args[]) {
  /*
   * Set the Nimbus look and feel
   */
  //<editor-fold defaultstate="collapsed" desc=" Look and feel setting code (optional) ">
  /*
   * If Nimbus (introduced in Java SE 6) is not available, stay with the default
   * look and feel. For details see
   * http://download.oracle.com/javase/tutorial/uiswing/lookandfeel/plaf.html
   */
  try {
   for (javax.swing.UIManager.LookAndFeelInfo info : javax.swing.UIManager
    .getInstalledLookAndFeels()) {
    if ("Nimbus".equals(info.getName())) {
     javax.swing.UIManager.setLookAndFeel(info.getClassName());
     break;
    }
   }
  } catch (ClassNotFoundException | InstantiationException
   | IllegalAccessException | javax.swing.UnsupportedLookAndFeelException ex) {
   java.util.logging.Logger.getLogger(OpenGLFrame.class.getName()).log(
    java.util.logging.Level.SEVERE, null, ex);
  }
  //</editor-fold>
  //</editor-fold>
  //</editor-fold>
  //</editor-fold>

  /*
   * Create and display the form
   */
  java.awt.EventQueue.invokeLater(() -> new OpenGLFrame().setVisible(true));
 }
 // Variables declaration - do not modify//GEN-BEGIN:variables
 private java.awt.Canvas canvas;
 // End of variables declaration//GEN-END:variables

 @Override
 public void mouseClicked(MouseEvent e) {
  mouse.mouseClicked(apply_grabbing_to_event(e));
 }

 @Override
 public void mousePressed(MouseEvent e) {
  mouse.mousePressed(apply_grabbing_to_event(e));
 }

 @Override
 public void mouseReleased(MouseEvent e) {
  mouse.mouseReleased(apply_grabbing_to_event(e));
 }

 @Override
 public void mouseEntered(MouseEvent e) {
  mouse.mouseEntered(apply_grabbing_to_event(e));
 }

 @Override
 public void mouseExited(MouseEvent e) {
  mouse.mouseExited(apply_grabbing_to_event(e));
 }

 @Override
 public void mouseDragged(MouseEvent e) {
  mouse_motion.mouseDragged(apply_grabbing_to_event(e));
 }

 @Override
 public void mouseMoved(MouseEvent e) {
  mouse_motion.mouseMoved(apply_grabbing_to_event(e));
 }

 @Override
 public void mouseWheelMoved(MouseWheelEvent e) {
  mouse_wheel.mouseWheelMoved((MouseWheelEvent) apply_grabbing_to_event(e));
 }

}
