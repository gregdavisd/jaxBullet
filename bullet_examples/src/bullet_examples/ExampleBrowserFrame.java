/*
 * Copyright (c) 2017 Gregery Barton
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

import bullet_examples.apps.api.TestHingeTorque;
import bullet_examples.apps.api.MotorDemo;
import bullet_examples.apps.api.GyroscopicDemo;
import bullet_examples.apps.api.BasicExample;
import bullet_examples.apps.api.AllConstraintDemo;
import bullet_examples.apps.api.RollingFrictionDemo;
import bullet_examples.apps.api.MotorizedHingeDemo;
import bullet_examples.apps.api.Dof6Spring2Demo;
import static Bullet.Collision.btIDebugDraw.DBG_DrawAabb;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraintLimits;
import static Bullet.Collision.btIDebugDraw.DBG_DrawConstraints;
import static Bullet.Collision.btIDebugDraw.DBG_DrawContactPoints;
import static Bullet.Collision.btIDebugDraw.DBG_DrawFrames;
import static Bullet.Collision.btIDebugDraw.DBG_DrawNormals;
import static Bullet.Collision.btIDebugDraw.DBG_DrawWireframe;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_USE_2_FRICTION_DIRECTIONS;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_USE_WARMSTARTING;
import Bullet.Dynamics.btContactSolverInfo;
import bullet_examples.apps.api.RigidBodySoftContact;
import bullet_examples.apps.benchmarks.ConvexStack;
import bullet_examples.apps.benchmarks.ConvexVsMesh;
import bullet_examples.apps.benchmarks.PrimVsMesh;
import bullet_examples.apps.benchmarks.RagDollsBenchmark;
import bullet_examples.apps.benchmarks.ThousandBoxes;
import bullet_examples.apps.benchmarks.ThousandStack;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.JPanel;
import javax.swing.JTree;
import javax.swing.SwingUtilities;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeModel;
import static javax.vecmath.VecMath.DEBUG_BLOCKS;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.PixelFormat;
import static Bullet.Dynamics.ConstraintSolver.btSolverMode.SOLVER_RANDOMIZE_ORDER;
import bullet_examples.apps.character.CharacterDemo;
import bullet_examples.apps.raycast.RaytestDemo;
import bullet_examples.apps.vehicle.ForkLiftDemo;

/**
 *
 * @author Gregery Barton
 */
public class ExampleBrowserFrame extends javax.swing.JFrame {

 static boolean reset = false;
 static boolean vsync = false;
 static boolean update_debug_flags = false;
 static float cap;
 private static String browser_selection;
 private static String running_demo;
 static DemoContainer demo;
 static final Object SYNC_DEMO = new Object();
 static float rate = 1.0f;
 static final String NODE_BASIC_EXAMPLE = "Basic Example";
 static final String NODE_ROLLING_FRICTION_DEMO = "Rolling Friction";
 static final String NODE_ALL_CONSTRAINTS = "Constraints";
 static final String NODE_MOTORIZED_HINGE = "Motorized Hinge";
 static final String NODE_TEST_HINGE_TORQUE = "TestHingeTorque";
 static final String NODE_6DOF_SPRING2 = "6DofSpring2";
 static final String NODE_MOTOR_DEMO = "Motor Demo";
 static final String NODE_GYROSCOPIC_DEMO = "Gyroscopic Demo";
 static final String NODE_RIGID_BODY_SOFT_CONTACT = "Soft Contact";
 static final String NODE_THOUSAND_BOXES = "3000 boxes";
 static final String NODE_THOUSAND_STACK = "1000 stack";
 static final String NODE_RAGDOLLS = "Rag Dolls";
 static final String NODE_CONVEX_STACK = "Convex Stack";
 static final String NODE_PRIM_VS_MESH = "Prim vs. Mesh";
 static final String NODE_CONVEX_VS_MESH = "Convex vs. Mesh";
 static final String NODE_FORK_LIFT = "Fork Lift";
 static final String NODE_RAY_TEST = "Raytest";
 static final String NODE_CHARACTER_DEMO = "Character Demo";
 static final String DEFAULT_DEMO = NODE_BASIC_EXAMPLE;
 private static boolean cycle;
 private static boolean activate_window = false;
 private static boolean backgrounded = false;
 private static int user_debug_mode = 0;
 private static int user_solver_mode = new btContactSolverInfo().m_solverMode;
 private static boolean user_split_impulse = new btContactSolverInfo().m_splitImpulse;
 private static final Throttle fps_throttle = new Throttle(1, 1);
 private static long last_frametime;
 private static int fps_samples;
 private static double fps;
 private static final long serialVersionUID = 1L;

 private static void update_debug_checks() {
  java.awt.EventQueue.invokeLater(new Runnable() {
   @Override
   public void run() {
    if (demo != null && demo.world() != null && demo.world().getDebugDrawer()
     != null) {
     int mode = demo.world().getDebugDrawer().getDebugMode();
     frame.aabb_check.setSelected((mode & DBG_DrawAabb) != 0);
     frame.wireframe_check.setSelected((mode & DBG_DrawWireframe) != 0);
     frame.constraint_limits_check.setSelected((mode & DBG_DrawConstraintLimits)
      != 0);
     frame.constraints_check.setSelected((mode & DBG_DrawConstraints) != 0);
     frame.frames_check.setSelected((mode & DBG_DrawFrames) != 0);
     frame.normals_check.setSelected((mode & DBG_DrawNormals) != 0);
     frame.contact_points_check.setSelected((mode & DBG_DrawContactPoints) != 0);
     btContactSolverInfo info = demo.world().getSolverInfo();
     frame.friction2_check.setSelected((info.m_solverMode
      & SOLVER_USE_2_FRICTION_DIRECTIONS) != 0);
     frame.warmstarting_check.setSelected((info.m_solverMode
      & SOLVER_USE_WARMSTARTING) != 0);
     frame.randomize_check.setSelected((info.m_solverMode
      & SOLVER_RANDOMIZE_ORDER) != 0);
     frame.interleave_check.setSelected((info.m_solverMode
      & SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS) != 0);
     frame.split_impulse_check.setSelected(info.m_splitImpulse);
    }
   }

  });
 }

 /**
  * Creates new form ExampleBrowserFrame
  */
 public ExampleBrowserFrame() {
  initComponents();
 }

 private static DemoContainer build_example(String selection) {
  final DemoContainer new_demo;
  switch (selection) {
   case NODE_BASIC_EXAMPLE:
    new_demo = new BasicExample();
    break;
   case NODE_ROLLING_FRICTION_DEMO:
    new_demo = new RollingFrictionDemo();
    break;
   case NODE_ALL_CONSTRAINTS:
    new_demo = new AllConstraintDemo();
    break;
   case NODE_MOTORIZED_HINGE:
    new_demo = new MotorizedHingeDemo();
    break;
   case NODE_TEST_HINGE_TORQUE:
    new_demo = new TestHingeTorque();
    break;
   case NODE_6DOF_SPRING2:
    new_demo = new Dof6Spring2Demo();
    break;
   case NODE_MOTOR_DEMO:
    new_demo = new MotorDemo();
    break;
   case NODE_GYROSCOPIC_DEMO:
    new_demo = new GyroscopicDemo();
    break;
   case NODE_RIGID_BODY_SOFT_CONTACT:
    new_demo = new RigidBodySoftContact();
    break;
   case NODE_THOUSAND_BOXES:
    new_demo = new ThousandBoxes();
    break;
   case NODE_THOUSAND_STACK:
    new_demo = new ThousandStack();
    break;
   case NODE_RAGDOLLS:
    new_demo = new RagDollsBenchmark();
    break;
   case NODE_CONVEX_STACK:
    new_demo = new ConvexStack();
    break;
   case NODE_PRIM_VS_MESH:
    new_demo = new PrimVsMesh();
    break;
   case NODE_CONVEX_VS_MESH:
    new_demo = new ConvexVsMesh();
    break;
   case NODE_FORK_LIFT:
    new_demo = new ForkLiftDemo();
    break;
   case NODE_RAY_TEST:
    new_demo = new RaytestDemo();
    break;
   case NODE_CHARACTER_DEMO:
    new_demo = new CharacterDemo();
    break;
   default:
    new_demo = null;
  }
  if (new_demo != null) {
   java.awt.EventQueue.invokeLater(new Runnable() {
    @Override
    public void run() {
     gl_frame.garbage_collect();
     frame.params_panel.removeAll();
     JPanel panel = new_demo.getParams();
     if (panel != null) {
      frame.params_panel.add(panel);
      panel.setVisible(true);
     }
     frame.description_text.setText(new_demo.get_description());
     SwingUtilities.updateComponentTreeUI(frame);
     gl_frame.set_mouse_listener(new_demo);
    }

   });
  }
  return new_demo;
 }

 /**
  * This method is called from within the constructor to initialize the form.
  * WARNING: Do NOT modify this code. The content of this method is always
  * regenerated by the Form Editor.
  */
 @SuppressWarnings("unchecked")
 // <editor-fold defaultstate="collapsed" desc="Generated Code">//GEN-BEGIN:initComponents
 private void initComponents() {

  buttonGroup1 = new javax.swing.ButtonGroup();
  root_panel = new javax.swing.JPanel();
  jPanel2 = new javax.swing.JPanel();
  jScrollPane2 = new javax.swing.JScrollPane();
  description_text = new javax.swing.JTextArea();
  jPanel3 = new javax.swing.JPanel();
  jTabbedPane1 = new javax.swing.JTabbedPane();
  jPanel5 = new javax.swing.JPanel();
  example_browser = new javax.swing.JTree();
  jPanel6 = new javax.swing.JPanel();
  jPanel4 = new javax.swing.JPanel();
  jScrollPane3 = new javax.swing.JScrollPane();
  jTextArea2 = new javax.swing.JTextArea();
  monitor_settings = new javax.swing.JPanel();
  vsync_check = new javax.swing.JCheckBox();
  cap60 = new javax.swing.JRadioButton();
  cap120 = new javax.swing.JRadioButton();
  cap30 = new javax.swing.JRadioButton();
  nocap = new javax.swing.JRadioButton();
  fps_label = new javax.swing.JLabel();
  jPanel7 = new javax.swing.JPanel();
  wireframe_check = new javax.swing.JCheckBox();
  aabb_check = new javax.swing.JCheckBox();
  contact_points_check = new javax.swing.JCheckBox();
  constraints_check = new javax.swing.JCheckBox();
  constraint_limits_check = new javax.swing.JCheckBox();
  normals_check = new javax.swing.JCheckBox();
  frames_check = new javax.swing.JCheckBox();
  jPanel8 = new javax.swing.JPanel();
  cycle_check = new javax.swing.JCheckBox();
  rate_slider = new javax.swing.JSlider();
  reset_button = new javax.swing.JButton();
  iterations_spinner = new javax.swing.JSpinner();
  broadphase_combo = new javax.swing.JComboBox<>();
  randomize_check = new javax.swing.JCheckBox();
  interleave_check = new javax.swing.JCheckBox();
  friction2_check = new javax.swing.JCheckBox();
  split_impulse_check = new javax.swing.JCheckBox();
  warmstarting_check = new javax.swing.JCheckBox();
  solver_combo = new javax.swing.JComboBox<>();
  params_panel = new javax.swing.JPanel();
  jMenuBar1 = new javax.swing.JMenuBar();
  jMenu1 = new javax.swing.JMenu();
  jMenu2 = new javax.swing.JMenu();

  setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
  setTitle("Bullet Physics Example Browser");
  setMaximumSize(new java.awt.Dimension(710, 650));
  setResizable(false);
  addWindowFocusListener(new java.awt.event.WindowFocusListener() {
   public void windowGainedFocus(java.awt.event.WindowEvent evt) {
    formWindowGainedFocus(evt);
   }
   public void windowLostFocus(java.awt.event.WindowEvent evt) {
   }
  });
  getContentPane().setLayout(new javax.swing.BoxLayout(getContentPane(), javax.swing.BoxLayout.LINE_AXIS));

  root_panel.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
  root_panel.setAlignmentX(0.0F);
  root_panel.setAlignmentY(0.0F);

  jPanel2.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));

  description_text.setColumns(20);
  description_text.setFont(new java.awt.Font("Arial", 0, 14)); // NOI18N
  description_text.setLineWrap(true);
  description_text.setRows(5);
  description_text.setWrapStyleWord(true);
  description_text.setPreferredSize(null);
  jScrollPane2.setViewportView(description_text);

  javax.swing.GroupLayout jPanel2Layout = new javax.swing.GroupLayout(jPanel2);
  jPanel2.setLayout(jPanel2Layout);
  jPanel2Layout.setHorizontalGroup(
   jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addComponent(jScrollPane2)
  );
  jPanel2Layout.setVerticalGroup(
   jPanel2Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addComponent(jScrollPane2, javax.swing.GroupLayout.DEFAULT_SIZE, 101, Short.MAX_VALUE)
  );

  jPanel3.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));

  example_browser.addTreeSelectionListener(new javax.swing.event.TreeSelectionListener() {
   public void valueChanged(javax.swing.event.TreeSelectionEvent evt) {
    example_browserValueChanged(evt);
   }
  });

  javax.swing.GroupLayout jPanel5Layout = new javax.swing.GroupLayout(jPanel5);
  jPanel5.setLayout(jPanel5Layout);
  jPanel5Layout.setHorizontalGroup(
   jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addComponent(example_browser, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, 205, Short.MAX_VALUE)
  );
  jPanel5Layout.setVerticalGroup(
   jPanel5Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addComponent(example_browser, javax.swing.GroupLayout.DEFAULT_SIZE, 433, Short.MAX_VALUE)
  );

  jTabbedPane1.addTab("Examples", jPanel5);

  javax.swing.GroupLayout jPanel6Layout = new javax.swing.GroupLayout(jPanel6);
  jPanel6.setLayout(jPanel6Layout);
  jPanel6Layout.setHorizontalGroup(
   jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGap(0, 205, Short.MAX_VALUE)
  );
  jPanel6Layout.setVerticalGroup(
   jPanel6Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGap(0, 433, Short.MAX_VALUE)
  );

  jTabbedPane1.addTab("Tests", jPanel6);

  javax.swing.GroupLayout jPanel3Layout = new javax.swing.GroupLayout(jPanel3);
  jPanel3.setLayout(jPanel3Layout);
  jPanel3Layout.setHorizontalGroup(
   jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel3Layout.createSequentialGroup()
    .addGap(0, 0, Short.MAX_VALUE)
    .addComponent(jTabbedPane1, javax.swing.GroupLayout.PREFERRED_SIZE, 207, javax.swing.GroupLayout.PREFERRED_SIZE))
  );
  jPanel3Layout.setVerticalGroup(
   jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addComponent(jTabbedPane1)
  );

  jPanel4.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
  jPanel4.setMaximumSize(new java.awt.Dimension(442, 32767));

  jTextArea2.setEditable(false);
  jTextArea2.setBackground(javax.swing.UIManager.getDefaults().getColor("Button.background"));
  jTextArea2.setColumns(20);
  jTextArea2.setFont(new java.awt.Font("Dialog", 0, 14)); // NOI18N
  jTextArea2.setForeground(javax.swing.UIManager.getDefaults().getColor("Button.focus"));
  jTextArea2.setLineWrap(true);
  jTextArea2.setText("WASD to move, Shift to quick move, left click to look, right click to interact. .");
  jTextArea2.setWrapStyleWord(true);
  jTextArea2.setAutoscrolls(false);
  jScrollPane3.setViewportView(jTextArea2);

  monitor_settings.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));

  vsync_check.setText("VSync");
  vsync_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    vsync_checkActionPerformed(evt);
   }
  });

  buttonGroup1.add(cap60);
  cap60.setText("60 cap");
  cap60.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    cap60ActionPerformed(evt);
   }
  });

  buttonGroup1.add(cap120);
  cap120.setText("120 cap");
  cap120.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    cap120ActionPerformed(evt);
   }
  });

  buttonGroup1.add(cap30);
  cap30.setText("30 cap");
  cap30.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    cap30ActionPerformed(evt);
   }
  });

  buttonGroup1.add(nocap);
  nocap.setText("no cap");
  nocap.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    nocapActionPerformed(evt);
   }
  });

  fps_label.setFont(new java.awt.Font("DialogInput", 3, 36)); // NOI18N
  fps_label.setHorizontalAlignment(javax.swing.SwingConstants.TRAILING);
  fps_label.setText("9999");
  fps_label.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));

  javax.swing.GroupLayout monitor_settingsLayout = new javax.swing.GroupLayout(monitor_settings);
  monitor_settings.setLayout(monitor_settingsLayout);
  monitor_settingsLayout.setHorizontalGroup(
   monitor_settingsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(monitor_settingsLayout.createSequentialGroup()
    .addGroup(monitor_settingsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
     .addComponent(vsync_check, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.PREFERRED_SIZE, 72, javax.swing.GroupLayout.PREFERRED_SIZE)
     .addComponent(cap120, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(cap30, javax.swing.GroupLayout.Alignment.LEADING)
     .addComponent(cap60, javax.swing.GroupLayout.Alignment.LEADING)
     .addComponent(nocap, javax.swing.GroupLayout.Alignment.LEADING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
    .addGap(0, 0, Short.MAX_VALUE))
   .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, monitor_settingsLayout.createSequentialGroup()
    .addContainerGap()
    .addComponent(fps_label)
    .addContainerGap())
  );
  monitor_settingsLayout.setVerticalGroup(
   monitor_settingsLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(monitor_settingsLayout.createSequentialGroup()
    .addComponent(vsync_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(cap30)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(cap60)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(cap120)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(nocap)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
    .addComponent(fps_label, javax.swing.GroupLayout.PREFERRED_SIZE, 34, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addContainerGap())
  );

  jPanel7.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));

  wireframe_check.setText("Wireframe");
  wireframe_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    wireframe_checkActionPerformed(evt);
   }
  });

  aabb_check.setText("AABB");
  aabb_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    aabb_checkActionPerformed(evt);
   }
  });

  contact_points_check.setText("Contact Points");
  contact_points_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    contact_points_checkActionPerformed(evt);
   }
  });

  constraints_check.setText("Constraints");
  constraints_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    constraints_checkActionPerformed(evt);
   }
  });

  constraint_limits_check.setText("Constraint limits");
  constraint_limits_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    constraint_limits_checkActionPerformed(evt);
   }
  });

  normals_check.setText("Normals");
  normals_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    normals_checkActionPerformed(evt);
   }
  });

  frames_check.setText("Frames");
  frames_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    frames_checkActionPerformed(evt);
   }
  });

  javax.swing.GroupLayout jPanel7Layout = new javax.swing.GroupLayout(jPanel7);
  jPanel7.setLayout(jPanel7Layout);
  jPanel7Layout.setHorizontalGroup(
   jPanel7Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addComponent(constraint_limits_check, javax.swing.GroupLayout.DEFAULT_SIZE, 127, Short.MAX_VALUE)
   .addComponent(normals_check, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
   .addComponent(constraints_check, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
   .addComponent(contact_points_check, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
   .addComponent(aabb_check, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
   .addComponent(wireframe_check, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
   .addComponent(frames_check, javax.swing.GroupLayout.PREFERRED_SIZE, 121, javax.swing.GroupLayout.PREFERRED_SIZE)
  );
  jPanel7Layout.setVerticalGroup(
   jPanel7Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(jPanel7Layout.createSequentialGroup()
    .addComponent(wireframe_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(aabb_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(contact_points_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(constraints_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(constraint_limits_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(normals_check)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(frames_check)
    .addGap(0, 0, Short.MAX_VALUE))
  );

  jPanel8.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
  jPanel8.setAlignmentX(0.0F);

  cycle_check.setText("Cycle");
  cycle_check.setToolTipText("Reset when all bodies are sleeping");
  cycle_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    cycle_checkActionPerformed(evt);
   }
  });

  rate_slider.setMajorTickSpacing(1);
  rate_slider.setMaximum(4);
  rate_slider.setMinimum(-4);
  rate_slider.setPaintLabels(true);
  rate_slider.setSnapToTicks(true);
  rate_slider.setToolTipText("Simulation rate");
  rate_slider.setValue(1);
  rate_slider.addChangeListener(new javax.swing.event.ChangeListener() {
   public void stateChanged(javax.swing.event.ChangeEvent evt) {
    rate_sliderStateChanged(evt);
   }
  });

  reset_button.setText("Reset");
  reset_button.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    reset_buttonActionPerformed(evt);
   }
  });

  iterations_spinner.setModel(new javax.swing.SpinnerNumberModel(10, 1, 256, 1));
  iterations_spinner.setToolTipText("Iterations");
  iterations_spinner.addChangeListener(new javax.swing.event.ChangeListener() {
   public void stateChanged(javax.swing.event.ChangeEvent evt) {
    iterations_spinnerStateChanged(evt);
   }
  });

  broadphase_combo.setModel(new javax.swing.DefaultComboBoxModel<>(new String[] { "btDbvtBroadphase", "btAxisSweep3" }));
  broadphase_combo.setToolTipText("Broadphase");
  broadphase_combo.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    broadphase_comboActionPerformed(evt);
   }
  });

  randomize_check.setText("Solver randomize");
  randomize_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    randomize_checkActionPerformed(evt);
   }
  });

  interleave_check.setText("Interleave contact/friction");
  interleave_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    interleave_checkActionPerformed(evt);
   }
  });

  friction2_check.setText("2 friction directions");
  friction2_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    friction2_checkActionPerformed(evt);
   }
  });

  split_impulse_check.setText("Split impulse");
  split_impulse_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    split_impulse_checkActionPerformed(evt);
   }
  });

  warmstarting_check.setText("Warmstarting");
  warmstarting_check.addActionListener(new java.awt.event.ActionListener() {
   public void actionPerformed(java.awt.event.ActionEvent evt) {
    warmstarting_checkActionPerformed(evt);
   }
  });

  solver_combo.setModel(new javax.swing.DefaultComboBoxModel<>(new String[] { "btSequentialImpulseConstraintSolver" }));

  javax.swing.GroupLayout jPanel8Layout = new javax.swing.GroupLayout(jPanel8);
  jPanel8.setLayout(jPanel8Layout);
  jPanel8Layout.setHorizontalGroup(
   jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(jPanel8Layout.createSequentialGroup()
    .addContainerGap()
    .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
     .addGroup(jPanel8Layout.createSequentialGroup()
      .addComponent(rate_slider, javax.swing.GroupLayout.PREFERRED_SIZE, 176, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(iterations_spinner, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(cycle_check, javax.swing.GroupLayout.PREFERRED_SIZE, 77, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(reset_button))
     .addGroup(jPanel8Layout.createSequentialGroup()
      .addComponent(interleave_check)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(split_impulse_check))
     .addGroup(jPanel8Layout.createSequentialGroup()
      .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.TRAILING, false)
       .addComponent(broadphase_combo, 0, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
       .addComponent(friction2_check))
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
       .addGroup(jPanel8Layout.createSequentialGroup()
        .addComponent(warmstarting_check)
        .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
        .addComponent(randomize_check))
       .addComponent(solver_combo, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))))
    .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
  );
  jPanel8Layout.setVerticalGroup(
   jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(jPanel8Layout.createSequentialGroup()
    .addContainerGap()
    .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
     .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
      .addComponent(cycle_check, javax.swing.GroupLayout.PREFERRED_SIZE, 30, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addComponent(reset_button)
      .addComponent(iterations_spinner, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
     .addComponent(rate_slider, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
     .addComponent(broadphase_combo, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
     .addComponent(solver_combo, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE))
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
    .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
     .addComponent(friction2_check)
     .addComponent(warmstarting_check)
     .addComponent(randomize_check))
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
    .addGroup(jPanel8Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
     .addComponent(interleave_check)
     .addComponent(split_impulse_check))
    .addContainerGap())
  );

  params_panel.setBorder(javax.swing.BorderFactory.createBevelBorder(javax.swing.border.BevelBorder.RAISED));
  params_panel.setLayout(new java.awt.BorderLayout());

  javax.swing.GroupLayout jPanel4Layout = new javax.swing.GroupLayout(jPanel4);
  jPanel4.setLayout(jPanel4Layout);
  jPanel4Layout.setHorizontalGroup(
   jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(jPanel4Layout.createSequentialGroup()
    .addContainerGap()
    .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
     .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel4Layout.createSequentialGroup()
      .addComponent(monitor_settings, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(jPanel7, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(params_panel, javax.swing.GroupLayout.PREFERRED_SIZE, 156, javax.swing.GroupLayout.PREFERRED_SIZE))
     .addComponent(jScrollPane3)
     .addComponent(jPanel8, javax.swing.GroupLayout.Alignment.TRAILING, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
    .addContainerGap())
  );
  jPanel4Layout.setVerticalGroup(
   jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, jPanel4Layout.createSequentialGroup()
    .addContainerGap()
    .addComponent(jScrollPane3, javax.swing.GroupLayout.PREFERRED_SIZE, 53, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(jPanel8, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addGroup(jPanel4Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
     .addComponent(params_panel, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(jPanel7, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(monitor_settings, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
    .addContainerGap())
  );

  javax.swing.GroupLayout root_panelLayout = new javax.swing.GroupLayout(root_panel);
  root_panel.setLayout(root_panelLayout);
  root_panelLayout.setHorizontalGroup(
   root_panelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(root_panelLayout.createSequentialGroup()
    .addContainerGap()
    .addGroup(root_panelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING, false)
     .addComponent(jPanel2, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addGroup(root_panelLayout.createSequentialGroup()
      .addComponent(jPanel3, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
      .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
      .addComponent(jPanel4, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)))
    .addContainerGap(javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
  );
  root_panelLayout.setVerticalGroup(
   root_panelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
   .addGroup(javax.swing.GroupLayout.Alignment.TRAILING, root_panelLayout.createSequentialGroup()
    .addContainerGap()
    .addGroup(root_panelLayout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
     .addComponent(jPanel3, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE)
     .addComponent(jPanel4, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, Short.MAX_VALUE))
    .addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
    .addComponent(jPanel2, javax.swing.GroupLayout.PREFERRED_SIZE, javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
    .addContainerGap())
  );

  getContentPane().add(root_panel);

  jMenu1.setText("File");
  jMenuBar1.add(jMenu1);

  jMenu2.setText("Edit");
  jMenuBar1.add(jMenu2);

  setJMenuBar(jMenuBar1);

  pack();
 }// </editor-fold>//GEN-END:initComponents

 private void user_set_debug_flag(boolean flag, int bit) {
  if (demo != null && demo.world() != null && demo.world().getDebugDrawer()
   != null) {
   int prev_mode = demo.world().getDebugDrawer().getDebugMode();
   demo.set_debug_flag(flag, bit);
   /*
    * store user changes to debug flags for restoring when another demo is
    * chosen
    *
    */
   System.out.println("1: " + user_debug_mode);
   if (prev_mode != demo.world().getDebugDrawer().getDebugMode()) {
    System.out.println("2: " + demo.world().getDebugDrawer().getDebugMode());
    if (flag) {
     user_debug_mode |= bit;
    } else {
     user_debug_mode &= ~bit;
    }
   }
   System.out.println("3: " + user_debug_mode);
   update_debug_flags = true;
  }
 }

 private void user_set_solver_mode(boolean flag, int bit) {
  if (flag) {
   user_solver_mode |= bit;
  } else {
   user_solver_mode &= ~bit;
  }
  update_debug_flags = true;
 }
 private void formWindowGainedFocus(java.awt.event.WindowEvent evt) {//GEN-FIRST:event_formWindowGainedFocus
  if (backgrounded) {
   activate_window = true;
  }
 }//GEN-LAST:event_formWindowGainedFocus

 private void warmstarting_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_warmstarting_checkActionPerformed
  user_set_solver_mode(frame.warmstarting_check.isSelected(),
   SOLVER_USE_WARMSTARTING);
 }//GEN-LAST:event_warmstarting_checkActionPerformed

 private void split_impulse_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_split_impulse_checkActionPerformed
  user_split_impulse = split_impulse_check.isSelected();
  update_debug_flags = true;
 }//GEN-LAST:event_split_impulse_checkActionPerformed

 private void friction2_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_friction2_checkActionPerformed
  user_set_solver_mode(frame.friction2_check.isSelected(),
   SOLVER_USE_2_FRICTION_DIRECTIONS);
 }//GEN-LAST:event_friction2_checkActionPerformed

 private void interleave_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_interleave_checkActionPerformed
  user_set_solver_mode(frame.interleave_check.isSelected(),
   SOLVER_INTERLEAVE_CONTACT_AND_FRICTION_CONSTRAINTS);
 }//GEN-LAST:event_interleave_checkActionPerformed

 private void randomize_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_randomize_checkActionPerformed
  user_set_solver_mode(frame.randomize_check.isSelected(),
   SOLVER_RANDOMIZE_ORDER);
 }//GEN-LAST:event_randomize_checkActionPerformed

 private void broadphase_comboActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_broadphase_comboActionPerformed
  reset = true;
 }//GEN-LAST:event_broadphase_comboActionPerformed

 private void iterations_spinnerStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_iterations_spinnerStateChanged
  if (demo != null && demo.world() != null && demo.world().getSolverInfo()
   != null) {
   set_world_iterations();
  }
 }//GEN-LAST:event_iterations_spinnerStateChanged

 private void reset_buttonActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_reset_buttonActionPerformed
  if (demo != null) {
   reset = true;
  }
 }//GEN-LAST:event_reset_buttonActionPerformed

 private void rate_sliderStateChanged(javax.swing.event.ChangeEvent evt) {//GEN-FIRST:event_rate_sliderStateChanged
  int value = rate_slider.getValue();
  if (value >= 0) {
   rate = value;
  } else {
   int d = (-value + 1);
   rate = 1.0f / (d * d);
  }
 }//GEN-LAST:event_rate_sliderStateChanged

 private void cycle_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_cycle_checkActionPerformed
  cycle = cycle_check.isSelected();
 }//GEN-LAST:event_cycle_checkActionPerformed

 private void frames_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_frames_checkActionPerformed
  user_set_debug_flag(frames_check.isSelected(), DBG_DrawFrames);
 }//GEN-LAST:event_frames_checkActionPerformed

 private void normals_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_normals_checkActionPerformed
  user_set_debug_flag(normals_check.isSelected(), DBG_DrawNormals);
 }//GEN-LAST:event_normals_checkActionPerformed

 private void constraint_limits_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_constraint_limits_checkActionPerformed
  user_set_debug_flag(constraint_limits_check.isSelected(),
   DBG_DrawConstraintLimits);
 }//GEN-LAST:event_constraint_limits_checkActionPerformed

 private void constraints_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_constraints_checkActionPerformed
  user_set_debug_flag(constraints_check.isSelected(), DBG_DrawConstraints);
 }//GEN-LAST:event_constraints_checkActionPerformed

 private void contact_points_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_contact_points_checkActionPerformed
  user_set_debug_flag(contact_points_check.isSelected(), DBG_DrawContactPoints);
 }//GEN-LAST:event_contact_points_checkActionPerformed

 private void aabb_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_aabb_checkActionPerformed
  user_set_debug_flag(aabb_check.isSelected(), DBG_DrawAabb);
 }//GEN-LAST:event_aabb_checkActionPerformed

 private void wireframe_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_wireframe_checkActionPerformed
  user_set_debug_flag(wireframe_check.isSelected(), DBG_DrawWireframe);
 }//GEN-LAST:event_wireframe_checkActionPerformed

 private void nocapActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_nocapActionPerformed
  if (nocap.isSelected()) {
   cap = 0.0f;
  }
 }//GEN-LAST:event_nocapActionPerformed

 private void cap30ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_cap30ActionPerformed
  if (cap30.isSelected()) {
   cap = 1.0f / 30.0f;
  }
 }//GEN-LAST:event_cap30ActionPerformed

 private void cap120ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_cap120ActionPerformed
  if (cap120.isSelected()) {
   cap = 1.0f / 120.0f;
  }
 }//GEN-LAST:event_cap120ActionPerformed

 private void cap60ActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_cap60ActionPerformed
  if (cap60.isSelected()) {
   cap = 1.0f / 60.0f;
  }
 }//GEN-LAST:event_cap60ActionPerformed

 private void vsync_checkActionPerformed(java.awt.event.ActionEvent evt) {//GEN-FIRST:event_vsync_checkActionPerformed
  vsync = vsync_check.isSelected();
 }//GEN-LAST:event_vsync_checkActionPerformed

 private void example_browserValueChanged(javax.swing.event.TreeSelectionEvent evt) {//GEN-FIRST:event_example_browserValueChanged
  Object s = example_browser.getSelectionPath().getLastPathComponent();
  browser_selection = s.toString();
 }//GEN-LAST:event_example_browserValueChanged

 private void set_world_iterations() {
  if (demo != null && demo.world() != null && demo.world().getSolverInfo()
   != null) {
   demo.world().getSolverInfo().m_numIterations = (Integer) iterations_spinner
    .getValue();
  }
 }

 private void update_world_iterations() {
  if (demo != null && demo.world() != null && demo.world().getSolverInfo()
   != null) {
   iterations_spinner.setValue(demo.world().getSolverInfo().m_numIterations);
  }
 }

 static ExampleBrowserFrame frame;
 static OpenGLFrame gl_frame;

 static {
  System.setProperty("org.lwjgl.util.Debug", "false");
  System.setProperty("org.lwjgl.util.NoChecks", "true");
 }

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
    if ("CDE/Motif".equals(info.getName())) {
     javax.swing.UIManager.setLookAndFeel(info.getClassName());
     break;
    }
   }
  } catch (ClassNotFoundException | InstantiationException
   | IllegalAccessException | javax.swing.UnsupportedLookAndFeelException ex) {
   java.util.logging.Logger.getLogger(ExampleBrowserFrame.class.getName()).log(
    java.util.logging.Level.SEVERE, null, ex);
  }
  //</editor-fold>
  /*
   * Create and display the form
   */
  java.awt.EventQueue.invokeLater(new Runnable() {
   @Override
   public void run() {
    frame = new ExampleBrowserFrame();
    frame.vsync_check.setSelected(vsync);
    frame.cap60.doClick();
    init_example_browser(frame.example_browser);
    frame.setVisible(true);
    if (DEBUG_BLOCKS) {
     frame.setTitle(frame.getTitle() + " : DEBUG mode");
    }
    boolean asserts = false;
    assert (asserts = true);
    if (asserts) {
     frame.setTitle(frame.getTitle() + " : assertions enabled");
    }
   }

   private void init_example_browser(JTree tree) {
    DefaultMutableTreeNode node_API = new DefaultMutableTreeNode("API");
    node_API.add(new DefaultMutableTreeNode(NODE_BASIC_EXAMPLE));
    node_API.add(new DefaultMutableTreeNode(NODE_ROLLING_FRICTION_DEMO));
    node_API.add(new DefaultMutableTreeNode(NODE_ALL_CONSTRAINTS));
    node_API.add(new DefaultMutableTreeNode(NODE_MOTORIZED_HINGE));
    node_API.add(new DefaultMutableTreeNode(NODE_TEST_HINGE_TORQUE));
    node_API.add(new DefaultMutableTreeNode(NODE_6DOF_SPRING2));
    node_API.add(new DefaultMutableTreeNode(NODE_MOTOR_DEMO));
    node_API.add(new DefaultMutableTreeNode(NODE_GYROSCOPIC_DEMO));
    node_API.add(new DefaultMutableTreeNode(NODE_RIGID_BODY_SOFT_CONTACT));
    DefaultMutableTreeNode node_benchmarks = new DefaultMutableTreeNode(
     "Benchmarks");
    node_benchmarks.add(new DefaultMutableTreeNode(NODE_THOUSAND_BOXES));
    node_benchmarks.add(new DefaultMutableTreeNode(NODE_THOUSAND_STACK));
    node_benchmarks.add(new DefaultMutableTreeNode(NODE_RAGDOLLS));
    node_benchmarks.add(new DefaultMutableTreeNode(NODE_CONVEX_STACK));
    node_benchmarks.add(new DefaultMutableTreeNode(NODE_PRIM_VS_MESH));
    node_benchmarks.add(new DefaultMutableTreeNode(NODE_CONVEX_VS_MESH));
    DefaultMutableTreeNode node_vehicles = new DefaultMutableTreeNode("Vehicle");
    node_vehicles.add(new DefaultMutableTreeNode(NODE_FORK_LIFT));
    DefaultMutableTreeNode node_raycast = new DefaultMutableTreeNode("Ray Cast");
    node_raycast.add(new DefaultMutableTreeNode(NODE_RAY_TEST));
    DefaultMutableTreeNode node_character = new DefaultMutableTreeNode(
     "Character");
    node_character.add(new DefaultMutableTreeNode(NODE_CHARACTER_DEMO));
    DefaultMutableTreeNode node_Examples = new DefaultMutableTreeNode("Examples");
    node_Examples.add(node_API);
    node_Examples.add(node_benchmarks);
    node_Examples.add(node_vehicles);
    node_Examples.add(node_raycast);
    node_Examples.add(node_character);
    DefaultTreeModel model = new DefaultTreeModel(node_Examples);
    tree.setModel(model);
    tree.setRootVisible(false);
    tree.expandRow(0);
   }

  });
  while (frame == null) {
   try {
    Thread.sleep(1000);
   } catch (InterruptedException ex) {
   }
  }
  try {
   gl_frame = new OpenGLFrame();
   gl_frame.setVisible(true);
   Display.create(new PixelFormat(0, 24, 0));
   Display.setParent(gl_frame.canvas());
   activate_window = true;
   boolean _vsync = !vsync;
   int _debug_mode = 0;
   browser_selection = DEFAULT_DEMO;
   last_frametime = System.nanoTime();
   while (!Display.isCloseRequested()) {
    synchronized (SYNC_DEMO) {
     backgrounded = (!frame.isActive() && !gl_frame.isActive());
     if (activate_window) {
      if (gl_frame != null) {
       java.awt.EventQueue.invokeLater(() -> {
        try {
         gl_frame.toFront();
        } catch (Exception ex) {
        } finally {
        }
       });
       activate_window = false;
      }
     }
     if (reset) {
      demo = null;
      reset = false;
     }
     boolean demo_changed = false;
     if (demo == null || browser_selection != null) {
      DemoContainer new_demo = null;
      if (browser_selection != null) {
       new_demo = build_example(browser_selection);
       if (new_demo != null) {
        demo_changed = true;
        running_demo = browser_selection;
       }
       browser_selection = null;
      }
      if (new_demo == null) {
       if (demo == null) {
        if (running_demo == null) {
         running_demo = DEFAULT_DEMO;
        }
        new_demo = build_example(running_demo);
       }
      }
      if (new_demo != null) {
       if (demo_changed) {
        new_demo.resetCamera();
        java.awt.EventQueue.invokeLater(() -> frame.update_world_iterations());
       }
       demo = new_demo;
       demo.initWorld((String) frame.broadphase_combo.getSelectedItem());
       demo.initPhysics();
       if (demo_changed) {
        user_split_impulse = demo.world().getSolverInfo().m_splitImpulse;
       } else {
        demo.world().getSolverInfo().m_splitImpulse = user_split_impulse;
       }
       new_demo.set_debug_mode(user_debug_mode | demo.world().getDebugDrawer()
        .getDebugMode());
       update_debug_flags = true;
       java.awt.EventQueue.invokeLater(() -> frame.set_world_iterations());
      }
     }
     if (vsync != _vsync) {
      _vsync = vsync;
      Display.setVSyncEnabled(_vsync);
     }
     if (update_debug_flags || _debug_mode != demo.world().getDebugDrawer()
      .getDebugMode() || user_solver_mode
      != demo.world().getSolverInfo().m_solverMode) {
      demo.world().getSolverInfo().m_solverMode = user_solver_mode;
      demo.world().getSolverInfo().m_splitImpulse = user_split_impulse;
      update_debug_checks();
      _debug_mode = demo.world().getDebugDrawer().getDebugMode();
      update_debug_flags = false;
     }
     if (demo.step_physics(cap, rate)) {
      boolean activity = demo.render_scene();
      if (cycle && !activity) {
       reset = true;
      }
      gl_frame.update_display();
      ++fps_samples;
      if (fps_throttle.update_now()) {
       long now = System.nanoTime();
       long duration = now - last_frametime;
       fps = fps_samples / (duration / 1e9);
       last_frametime = now;
       fps_samples = 0;
       java.awt.EventQueue.invokeLater(new Runnable() {
        @Override
        public void run() {
         String t = String.format("%04d", ((int) (fps + 0.5f)));
         frame.fps_label.setText(t);
        }

       });
      }
     }
    }
   }
   System.exit(0);
  } catch (Exception ex) {
   Logger.getLogger(ExampleBrowserFrame.class.getName()).log(Level.SEVERE, null,
    ex);
  }
  //</editor-fold>
 }
 // Variables declaration - do not modify//GEN-BEGIN:variables
 private javax.swing.JCheckBox aabb_check;
 private javax.swing.JComboBox<String> broadphase_combo;
 private javax.swing.ButtonGroup buttonGroup1;
 private javax.swing.JRadioButton cap120;
 private javax.swing.JRadioButton cap30;
 private javax.swing.JRadioButton cap60;
 private javax.swing.JCheckBox constraint_limits_check;
 private javax.swing.JCheckBox constraints_check;
 private javax.swing.JCheckBox contact_points_check;
 private javax.swing.JCheckBox cycle_check;
 private javax.swing.JTextArea description_text;
 private javax.swing.JTree example_browser;
 private javax.swing.JLabel fps_label;
 private javax.swing.JCheckBox frames_check;
 private javax.swing.JCheckBox friction2_check;
 private javax.swing.JCheckBox interleave_check;
 private javax.swing.JSpinner iterations_spinner;
 private javax.swing.JMenu jMenu1;
 private javax.swing.JMenu jMenu2;
 private javax.swing.JMenuBar jMenuBar1;
 private javax.swing.JPanel jPanel2;
 private javax.swing.JPanel jPanel3;
 private javax.swing.JPanel jPanel4;
 private javax.swing.JPanel jPanel5;
 private javax.swing.JPanel jPanel6;
 private javax.swing.JPanel jPanel7;
 private javax.swing.JPanel jPanel8;
 private javax.swing.JScrollPane jScrollPane2;
 private javax.swing.JScrollPane jScrollPane3;
 private javax.swing.JTabbedPane jTabbedPane1;
 private javax.swing.JTextArea jTextArea2;
 private javax.swing.JPanel monitor_settings;
 private javax.swing.JRadioButton nocap;
 private javax.swing.JCheckBox normals_check;
 private javax.swing.JPanel params_panel;
 private javax.swing.JCheckBox randomize_check;
 private javax.swing.JSlider rate_slider;
 private javax.swing.JButton reset_button;
 private javax.swing.JPanel root_panel;
 private javax.swing.JComboBox<String> solver_combo;
 private javax.swing.JCheckBox split_impulse_check;
 private javax.swing.JCheckBox vsync_check;
 private javax.swing.JCheckBox warmstarting_check;
 private javax.swing.JCheckBox wireframe_check;
 // End of variables declaration//GEN-END:variables
}
