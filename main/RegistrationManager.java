package artisynth.models.swallowingRegistrationTool.main;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import javax.swing.JButton;
import javax.swing.JMenuItem;
import javax.swing.JSeparator;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.modelbase.RenderableModelBase;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.correspondences.CloudMap;
import artisynth.models.swallowingRegistrationTool.transformers.HasSlaves;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import artisynth.models.swallowingRegistrationTool.infoUtilities.InfoAllocator;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.optimizers.Optimizer;
import artisynth.models.swallowingRegistrationTool.transformers.FFDPointNotFoundException;
import artisynth.models.swallowingRegistrationTool.transformers.HasAction;
import artisynth.models.swallowingRegistrationTool.transformers.HasRenderableModel;
import maspack.geometry.MeshBase;
import maspack.geometry.GeometryTransformer.UndoState;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.properties.PropertyList;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderable;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

public class RegistrationManager extends RenderableModelBase implements 
ActionListener{

   private HashMap <RegistrationTransformer, Optimizer> myActions
   = new LinkedHashMap <RegistrationTransformer, Optimizer>();
   private IterativeActions myIAs;

   protected ControlPanel myPanel;
   private NumericOutputProbe myprobe;
   JButton updateButton = new JButton ("Update");
   JButton updateMapButton = new JButton ("Update Map");

   private String [] menuItemNames;
   private ArrayList <String> menuSlaveActionItemNames = new ArrayList<String>();
   private boolean actionVisible = true;
   private boolean iterative = false;
   private boolean oldIterative = false;
   protected double myErr = 0.0;
   protected double myMaxErr = 0.0;

   CloudMap myMap;

   MeshComponentList <MeshModelAgent> mySrcAgent = new MeshComponentList 
   <MeshModelAgent> (MeshModelAgent.class, "Source Meshes", "SMs");
   MeshComponentList <MeshModelAgent> myTgtAgent = new MeshComponentList 
   <MeshModelAgent> (MeshModelAgent.class, "Target Meshes", "STs");
   ArrayList <MeshBase> mySrcs = new ArrayList<MeshBase> ();
   ArrayList <MeshBase> myTgts = new ArrayList<MeshBase> ();

   HashMap<TransformableGeometry, SlaveInfo> mySlaves = 
   new LinkedHashMap<TransformableGeometry, SlaveInfo> ();

   private boolean isMapValid = false;
   private boolean optimized = false;

   /**
    * Iterative actions will be advanced by root model.
    * 
    */
   protected void takeIterativeActions () {

      optimized = false;

      if (myIAs == null) {
         throw new UnsupportedOperationException (
         "Iterative actions not defined!");
      }
      // pick transformer
      RegistrationTransformer tf = myIAs.pickAction (getTransformerList ());
      if (tf != null) {
         // get optimizer 
         Optimizer opt = getOptimizer (tf);

         try {
            // add slaves
            Set<TransformableGeometry> slaves = new HashSet<TransformableGeometry> ();
            if (myIAs.addSlave (slaves)) {
               for (TransformableGeometry slave : slaves) {
                  addSlave (slave);
               }
            }
         }
         catch (Exception e) {
            e.printStackTrace ();
            System.err.println ("Failed to add slaves!");
         }

         // pick active slaves
         Set<SlaveInfo> activeSlaves = new HashSet<SlaveInfo> ();
         try {
            myIAs.pickSlaves (activeSlaves, mySlaves.values ());
            tf.shiftSlaves (activeSlaves);
         }
         catch (Exception e) {
            e.printStackTrace ();
            System.err.println ("Failed to pick active slaves");
         }

         boolean doAction = false;

         try {
            doAction = myIAs.preAction(this);
         }
         catch (Exception e) {
            e.printStackTrace ();
            System.err.println ("Failed to do preAction");
            System.err.println ("Jump to post update!");
         }

         if (doAction) {  
            // update formulator 
            Formulator F = null;
            try {
               F = myIAs.getFormulator ();
               F.setCloudMap (myMap);
               F.setTransformer (tf);
            }
            catch (Exception e) {
               e.printStackTrace ();
               System.err.println ("Failed to retrieve formulator!");
               System.err.println ("Jump to post update!");
               doAction = false;
            }

            if (doAction) {
               try {
                  optimized = optimizeAction (tf, F);
               }
               catch (Exception e) {
                  e.printStackTrace ();
                  System.err.println ("Failed to optimize!");
                  System.err.println ("Jump to post update!");
               }
            }
            
            try {
               doAction = myIAs.postOptimize (this, optimized);
            }
            catch (Exception e) {
               e.printStackTrace ();
               System.err.println ("Failed to do post optimize!");
               System.err.println ("Jump to post update!");
               doAction = optimized;
            }
            
            if (doAction) {
               try {
                  isMapValid = takeAction (tf);
               }
               catch (Exception e) {
                  e.printStackTrace ();
                  System.err.println ("Failed to take action!");
               }
               
               if (tf.isFastTransformer ()) {
                  try {
                     tf.fastTransformer (activeSlaves);
                  }
                  catch (Exception e) {
                     e.printStackTrace ();
                     System.err.println ("Failed to do fast transform");
                     System.err.println ("Do transform following the standard scheme!");
                  }
               }
               try {
                  for (SlaveInfo info : activeSlaves) {
                     takeTransform (tf, info.getSlave ());
                  }
               }
               catch (Exception e) {
                  e.printStackTrace ();
                  System.err.println ("Failed to do transform!");
               }
            }
         }
      }
      
      try {
         if ( ! myIAs.postUpdate (this)) {
            resetTime ();
         }
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println ("Failed to do post update!");
      }
      
   }

   public boolean isMapValid () {
      return isMapValid;
   }

   public boolean isOptimized () {
      return optimized;
   }

   /**
    * Formulate cost function for optimization; This method will be called in  
    * every iteration;
    * 
    * @param activeTf active transformer which is picked at the beginning of 
    * iterative action in every iteration;
    * 
    * @param activeOpt corresponding optimizer of the active transformer
    * @param activeSlaves slaves which will be transformed by the active transformer 
    * after optimization.
    */
   //public abstract Formulator formulate (RegistrationTransformer activeTf, 
   //Optimizer activeOpt, Set<SlaveInfo> activeSlaves);

   // ------------------------- initialize registration-----------------------------//

   public void setIterativeAction (IterativeActions actions) {
      myIAs = actions;
   }

   public IterativeActions getIterativeActions () {
      return myIAs;
   }

   public void setCloudMap (CloudMap map) {
      myMap = map;
   }

   public CloudMap getCloudMap () {
      return myMap;
   }

   /**
    * Allocating correspondences to <code>CloudMap</code>; Usually during 
    * this process, both correspondences creation and allocation are involved.
    * 
    * @param allocator allocator which is specified by users
    * @throws ReflectiveOperationException when failed to create a new 
    * <code>Correspondence</code>
    */
   public void allocateCorrespondences (InfoAllocator allocator) throws 
   ReflectiveOperationException {
      allocator.allocateInfo (myMap);
      isMapValid = true;
   }

   /**
    * Allocating correspondences to <code>CloudMap</code>; Usually during 
    * this process, both correspondences creation and allocation are involved.
    * 
    * @param allocators allocators which are specified by users
    * @throws ReflectiveOperationException when failed to create a new 
    * <code>Correspondence</code>
    */
   public void allocateCorrespondences (InfoAllocator [] allocators) throws 
   ReflectiveOperationException {
      allocators[0].allocateInfo (myMap);
      for (int i = 1; i < allocators.length; i++ ) {
         InfoAllocator allo = allocators[i];
         allo.addInfo (myMap);
      }
   }

   public void addMeshMatchPair(MeshBase src, MeshBase tgt) { 
      MeshModelAgent srcMeshAgent = new MeshModelAgent ();
      mySrcAgent.add (srcMeshAgent);
      mySrcs.add (src);
      srcMeshAgent.setMesh (src);
      srcMeshAgent.getMesh ().setFixed (false);
      


      MeshModelAgent tgtMeshAgent = new MeshModelAgent ();
      myTgtAgent.add (tgtMeshAgent);
      myTgts.add (tgt);
      tgtMeshAgent.setMesh (tgt);
      tgtMeshAgent.getMesh ().setFixed (false);
   }

   public void addMeshMatchPairs(List<? extends MeshBase> srcs, List<? extends MeshBase> tgts) { 
      if (srcs.size () != tgts.size ()) {
         throw new ImproperSizeException("Incompatible mesh list pair!");
      }
      for (int i = 0; i < srcs.size (); i++) {
         addMeshMatchPair(srcs.get (i), tgts.get (i));
      }

   }

   public void addMeshMatchPairs(Map<MeshBase, MeshBase> Src2TgtMap) { 
      Set<Entry<MeshBase, MeshBase>> ens = Src2TgtMap.entrySet ();
      Iterator it = ens.iterator ();

      while (it.hasNext ()) {
         Entry<MeshBase, MeshBase> me = (Entry<MeshBase, MeshBase>)it.next ();
         addMeshMatchPair (me.getKey (), me.getValue ());
      }
   }

   /**
    * render target and source mesh
    * @param root
    */
   public void renderSourceAndTargetMesh(RootModel root) {
      RenderProps.setFaceStyle (mySrcAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(mySrcAgent, Color.RED);
      RenderProps.setAlpha(mySrcAgent, 1);
      
      RenderProps.setFaceStyle (myTgtAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(myTgtAgent, Color.BLUE);
      RenderProps.setAlpha(myTgtAgent, 1);
      
      root.addRenderable (mySrcAgent);
      root.addRenderable (myTgtAgent);
   }
   
   /**
    * render target and source mesh
    * @param root
    */
   public void renderSourceAndTargetMesh(RootModel root, double alpha) {
      RenderProps.setFaceStyle (mySrcAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(mySrcAgent, Color.RED);
      RenderProps.setAlpha(mySrcAgent, alpha);
      
      RenderProps.setFaceStyle (myTgtAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(myTgtAgent, Color.BLUE);
      RenderProps.setAlpha(myTgtAgent, alpha);
      root.addRenderable (mySrcAgent);
      root.addRenderable (myTgtAgent);
   }
   
   public void renderSourceAndTargetMesh(RootModel root, 
      double srcAlpha, double tgtAlpha, 
      Shading srcShadeType, Shading tgtShadeType) {
      RenderProps.setFaceStyle (mySrcAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(mySrcAgent, Color.RED);
      RenderProps.setAlpha(mySrcAgent, srcAlpha);
      RenderProps.setShading (mySrcAgent, srcShadeType);
      
      RenderProps.setFaceStyle (myTgtAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(myTgtAgent, Color.BLUE);
      RenderProps.setAlpha(myTgtAgent, tgtAlpha);
      RenderProps.setShading (myTgtAgent, tgtShadeType);
      root.addRenderable (mySrcAgent);
      root.addRenderable (myTgtAgent);
   }

   public void renderSourceAndTargetMesh(RootModel root, double srcAlpha, 
      double tgtAlpha, Shading shadeType) {
      RenderProps.setFaceStyle (mySrcAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(mySrcAgent, Color.RED);
      RenderProps.setAlpha(mySrcAgent, srcAlpha);
      RenderProps.setShading (mySrcAgent, shadeType);
      
      RenderProps.setFaceStyle (myTgtAgent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setFaceColor(myTgtAgent, Color.BLUE);
      RenderProps.setAlpha(myTgtAgent, tgtAlpha);
      RenderProps.setShading (myTgtAgent, shadeType);
      root.addRenderable (mySrcAgent);
      root.addRenderable (myTgtAgent);
   }
   
   public void disableRenderSourceAndTargetMesh (RootModel root) {
      if (root.renderables ().contains (mySrcAgent)) {
         root.removeRenderable (mySrcAgent);
      }
      if (root.renderables ().contains (myTgtAgent)) {
         root.removeRenderable (myTgtAgent);
      }
   }

   public void getSourceMeshes (ArrayList<MeshBase> srcs) {
      srcs.addAll (mySrcs);
   }

   public void getTargetMeshes (ArrayList<MeshBase> tgt) {
      tgt.addAll (myTgts);
   }

   public ArrayList<MeshBase> getSourceMeshes () {
      return mySrcs;
   }

   public ArrayList<MeshBase> getTargetMeshes () {
      return myTgts;
   }
   
   public void setSourceMeshRenderProps (RenderProps props) {
      mySrcAgent.setRenderProps (props);
   }
   
   public void setTargetMeshRenderProps (RenderProps props) {
      myTgtAgent.setRenderProps (props);
   }

   private boolean saveEdge = false;
   private boolean saveStiffness = false;
   private boolean saveElement = false;
   boolean warpElements = false;
   private boolean updateCfMd = false;
   private boolean saveElementStiffness = false;

   public void enableSlaveEdgeSaving (boolean enable) {
      saveEdge = enable;
   }

   public boolean isSlaveEdgeSavingEnabled () {
      return saveEdge;
   }

   public void enableSlaveStiffnessSaving (boolean enable) {
      saveStiffness = enable;
   }

   public boolean isSlaveStiffnessSavingEnabled () {
      return saveStiffness;
   }

   public void enableSlaveElementSaving (boolean enable) {
      saveElement = enable;
   }

   public boolean isSlaveElementSavingEnabled () {
      return saveElement;
   }

   public void enableSlaveElementWarping (boolean enable) {
      warpElements = enable;
   }

   public boolean isSlaveElementWarpingEnabled () {
      return warpElements;
   }
   
   public void enableConformalModesUpdate (boolean enable) {
      updateCfMd = enable;
   }

   public boolean isConformalModesUpdateEnabled () {
      return updateCfMd;
   }
   
   public void enableElementStiffnessSaving (boolean enable) {
      saveElementStiffness = enable;
   }
   
   public boolean isElementStiffnessSavingEnabled () {
      return saveElementStiffness;
   }

   public boolean containSlave (TransformableGeometry slave) {
      if (mySlaves.containsKey (slave)) {
         return true;
      }
      return false;
   }

   public void addSlave (TransformableGeometry slave) {
      if (mySlaves.containsKey (slave)) {
         return;
      }
      SlaveInfo info = new SlaveInfo ();
      info.setSlave (slave);
      info.enableEdgeSaving (saveEdge);
      info.enableElementSaving (saveElement);
      info.enableStiffnessSaving (saveStiffness);
      info.enableElementWarping (warpElements);
      info.enableElementStiffnessSaving (saveElementStiffness);
      if (info.hasPoint ()) {
         info.savePoints ();
         info.savePointCurretAsUndeformed ();
      }
      info.update ();
      info.enableConformalModesUpdate (updateCfMd);
      mySlaves.put (slave, info);
   }

   public SlaveInfo getSlaveInfo (TransformableGeometry slave) {
      return mySlaves.get (slave);
   }

   public void assignSlaveInfo (TransformableGeometry slave, 
      RegistrationTransformer tf) {

      SlaveInfo info = mySlaves.get (slave);
      if (info == null) {
         throw new IllegalArgumentException ("not my slave!");
      }
      tf.addSlave (info);
   }

   /**
    * assign this slave to all <code>RegistrationTransformer</code>s
    * @param slave
    */
   public void assignSlaveInfo (TransformableGeometry slave) {
      SlaveInfo info = mySlaves.get (slave);
      if (info == null) {
         throw new IllegalArgumentException ("not my slave!");
      }
      for (RegistrationTransformer tf : myActions.keySet ()) {
         tf.addSlave (info);
      }
   }

   /**
    * assign all slaves to transformer <tt>tf</tt>
    * @param tf
    */
   public void assignSlaveInfo (RegistrationTransformer tf) {
      Set ens = mySlaves.entrySet ();
      Iterator it = ens.iterator ();
      while (it.hasNext ()) {
         Entry<TransformableGeometry, SlaveInfo> me = 
         (Entry<TransformableGeometry, SlaveInfo>)it.next ();
         tf.addSlave (me.getValue ());
      }
   }

   /**
    * assign all slaves to every <code>RegistrationTransformer<code>
    */
   public void assignSlaveInfo () {
      for (RegistrationTransformer tf : myActions.keySet ()) {
         assignSlaveInfo (tf);
      }
   }

   public List<SlaveInfo> getSlaves () {
      Set entries = mySlaves.entrySet ();
      Iterator it = entries.iterator ();
      
      List<SlaveInfo> slaves = new ArrayList<SlaveInfo> ();
      while (it.hasNext ()) {
         Entry<TransformableGeometry, SlaveInfo> me = 
         (Entry<TransformableGeometry, SlaveInfo>) it.next ();
         
         slaves.add (me.getValue ());
      }
      
      return slaves;
   }


   // ------------------------- constructor ------------------------------//

   public RegistrationManager (String name) {
      super(name);
      updateMapButton.addActionListener (this);
      updateButton.addActionListener (this);
   }


   // ------------------------- properties ------------------------------//

   public static PropertyList myProps =
   new PropertyList(RegistrationManager.class, RenderableModelBase.class);

   static {
      myProps.addReadOnly(
         "regErr", "mean error of registration");
      myProps.addReadOnly (
         "maxRegErr", "maximum error of registration");
      myProps.add ("enableIteration", "enable iterative action", true);
      myProps.add ("actionVisibility", "visibility of actions", true);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   public double getRegErr() {
      myErr = myMap.computeMappingError ();
      return myErr;
   }
   
   public double getMaxRegErr() {
      myMaxErr = myMap.computeMaxMappingError ();
      return myMaxErr;
   }

   public boolean getActionVisibility () {
      return actionVisible;
   }

   public void setEnableIteration (boolean enable) {
      iterative = enable;
   }

   public boolean getEnableIteration () {
      return iterative;
   }

   public void setActionVisibility (boolean visible) {
      actionVisible = visible;
   }

   // ------------------------- probe ------------------------------//
   /**
    * create an output probe for registration error. If <tt>root</tt> 
    * is not null, add this probe into that root model. The output file 
    * will be saved in /data directory of <tt>root</tt>.
    * 
    * @param root RootModel
    * @return the output probe for registration error
    */
   public NumericOutputProbe createRegistrationErrorProbe (RootModel root) {

      if (root != null) {
         myprobe = new NumericOutputProbe(this, "regErr", 
            ArtisynthPath.getSrcRelativePath (root, "/data/outputprobe_RegError.txt"), 0.01);
         root.addOutputProbe (myprobe);
      } else {
         myprobe = new NumericOutputProbe(this, "regErr", 
            ArtisynthPath.getSrcRelativePath (this, "/outputprobe_RegError.txt"), 0.01);
      }
      return myprobe;
   }

   public NumericOutputProbe getRegistrationErrorProbe () {
      return myprobe;
   }

   // ------------------------- panel ------------------------------//

   public ControlPanel createRegistrationControlPanel() {
      myPanel = new ControlPanel(getName() + " (Registration Manager)");
      myPanel.addWidget ("Mean Error", this, "regErr");
      myPanel.addWidget ("Max Error", this, "maxRegErr");
      myPanel.addWidget ("Iterate", this, "enableIteration");
      myPanel.addWidget ("Action Visibility", this, "actionVisibility");
      myPanel.addWidget (updateButton);
      myPanel.addWidget (updateMapButton);
      return myPanel;
   }

   public ControlPanel getRegistrationControlPanel () {
      return myPanel;
   }

   /**
    * add registration widges to <tt>panel</tt>;
    * @param panel
    */
   public void addWidgetsToControlPanel (ControlPanel panel) {
      if (panel.numWidgets () != 0) panel.addWidget (new JSeparator ());
      if (getName () != null) panel.addLabel (getName ());
      else panel.addLabel ("Registration " + getClass().getSimpleName ());
      panel.addWidget ("Total Erro", this, "regErr");
      panel.addWidget ("Max Erro", this, "maxRegErr");
      panel.addWidget ("Iterate", this, "enableIteration");
      panel.addWidget ("Action Visibility", this, "actionVisibility");
      panel.addWidget (updateButton);
      panel.addWidget (updateMapButton);
   }

   // ------------------------- actions ------------------------------//

   /**
    * add registration action into action-list
    * @param gt geometry transformer 
    * @param opt corresponding optimizer 
    * use to find best geometry transformer
    * @return old optimizer associates with <tt>gt</tt>
    * old optimizer will be replaced by 
    * new optimizer <tt>opt</tt>.
    */
   public Optimizer addAction (RegistrationTransformer gt, Optimizer opt) {
      if (! opt.checkSink (gt)) {
         throw new IllegalArgumentException(
         "Transformer is incompatible with optimizer !");
      }
      Optimizer oldTer = myActions.put (gt, opt);
      System.out.println("Action " + gt.getClass ().getSimpleName () +
         " - " + opt.getClass ().getSimpleName () +
      " has been add to registration action list! ");
      return oldTer;
   }

   /**
    * optimizer associated with action <tt>transformer</tt> 
    * will be called to optimize <tt>transformer</tt>; correspondences
    * will be taken from <code>CloudInfo</code> of this registration
    * manager;
    * @param transformer 
    * 
    * @return success to optimize return true otherwise return false
    */
   public boolean optimizeAction (RegistrationTransformer transformer, Formulator form) {
      if (! myActions.containsKey (transformer)) {
         throw new ImproperStateException (
         "Transformer is not in registrantion action list");
      }
      Optimizer opt = myActions.get (transformer);
      System.out.println ("Optimizing Transformer ...");
      boolean flag = false;
      try {
         flag = opt.optimize (transformer, form);
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println ("Failed to optimize");
         return false;
      }

      System.out.println ("Optimization done!");
      return flag;
   }

   /**
    * 
    * this will take the best registration action on geometry <tt>obj</tt>; 
    * optimizer associated with the <tt>transformer</tt> will be called
    *  to find the best action.
    * @param transformer 
    * @param obj transformable geometry
    * @param cloud-information needed to find the best registration action
    * @return if update cloud map successfully return true otherwise return false;
    */
   public boolean takeAction (RegistrationTransformer transformer) {
      if (! myActions.containsKey (transformer)) {
         throw new ImproperStateException (
         "Transformer is not in registrantion action list");
      }
      Optimizer opt = myActions.get (transformer);

      transformer.applyAction ();
      return myMap.update ();
   }


   /**
    * apply registration action directly, and transform geometry <tt>obj</tt> 
    * with this <tt>transformer</tt> directly.
    * @param transformer
    * @param obj
    */
   public void takeAction(RegistrationTransformer transformer, TransformableGeometry obj) {
      if (obj != null) {
         System.out.println ("Transforming geometry ...");
         transformer.setUndoState (UndoState.SAVING);
         TransformGeometryContext.transform (obj, transformer, 0);
      }
      if (! this.getTransformerList ().contains (transformer)) {
         throw new NullPointerException("Transformer not in registration action list");
      }
      if (HasAction.class.isAssignableFrom (transformer.getClass ())) {
         System.out.println ("Applying registration action ...");
         HasAction ha = (HasAction)transformer;
         ha.applyAction ();
         myMap.update ();
      }
      myMap.update ();
      System.out.println ("Action done !");
   }

   public void takeActionWithoutSaving(RegistrationTransformer transformer, TransformableGeometry obj) {
      if (obj != null) {
         System.out.println ("Transforming geometry ...");
         transformer.setUndoState (UndoState.OFF);
         TransformGeometryContext.transform (obj, transformer, 0);
      }
      if (! this.getTransformerList ().contains (transformer)) {
         throw new NullPointerException("Transformer not in registration action list");
      }
      if (HasAction.class.isAssignableFrom (transformer.getClass ())) {
         System.out.println ("Applying registration action ...");
         HasAction ha = (HasAction)transformer;
         ha.applyAction ();
      }
      myMap.update ();
      System.out.println ("Action done !");
   }

   /**
    * reset registration action and geometry
    * @param transformer
    * @param obj
    */
   public void resetAction(RegistrationTransformer transformer, TransformableGeometry obj) {
      if (obj != null) {
         System.out.println ("Restoring geometry ...");
         transformer.setUndoState (UndoState.RESTORING);
         TransformGeometryContext.transform (obj, transformer, 0);
      }
      if (HasAction.class.isAssignableFrom (transformer.getClass ())) {
         System.out.println ("Reseting registration action ...");
         HasAction ha = (HasAction)transformer;
         ha.resetAction ();
      }
      myMap.update ();
      System.out.println ("Action reseting done !");
   }

   public void takeTransform(RegistrationTransformer transformer, TransformableGeometry obj) {
      if (obj != null) {
         System.out.println ("transforming geometry ...");
         transformer.setUndoState (UndoState.SAVING);
         TransformGeometryContext.transform (obj, transformer, 0);
      }
   }

   public void takeTransformWithoutSaving(RegistrationTransformer transformer, TransformableGeometry obj) {
      if (obj != null) {
         System.out.println ("transforming geometry ...");
         transformer.setUndoState (UndoState.OFF);
         TransformGeometryContext.transform (obj, transformer, 0);
      }
   }



   public void restoreTransform(RegistrationTransformer transformer, TransformableGeometry obj) {
      if (obj != null) {
         System.out.println ("Restoring geometry ...");
         transformer.setUndoState (UndoState.RESTORING);
         TransformGeometryContext.transform (obj, transformer, 0);
      }
   }


   // ------------------------- get and set actions ------------------------------//

   public RegistrationTransformer getTransformer(int idx) {
      if (idx >= myActions.size ()) {
         throw new IndexOutOfBoundsException("False index");
      }
      return getTransformers()[idx];
   }

   public RegistrationTransformer[] getTransformers() {
      Set set = myActions.entrySet ();
      Iterator it = set.iterator ();

      RegistrationTransformer [] tfs = new RegistrationTransformer [myActions.size ()];
      int idx = 0;
      while (it.hasNext ()) {
         Entry<RegistrationTransformer, Optimizer> me = 
         (Entry<RegistrationTransformer, Optimizer>)it.next ();
         tfs[idx] = me.getKey ();
         idx++;
      }

      return tfs;
   }

   public ArrayList<RegistrationTransformer> getTransformerList() {
      Set set = myActions.entrySet ();
      Iterator it = set.iterator ();

      ArrayList<RegistrationTransformer> tlist = new ArrayList<RegistrationTransformer> ();
      while (it.hasNext ()) {
         Entry<RegistrationTransformer, Optimizer> me = 
         (Entry<RegistrationTransformer, Optimizer>)it.next ();
         tlist.add (me.getKey ());
      }

      return tlist;
   }

   public HashMap <RegistrationTransformer, Optimizer> getActions() {
      return myActions;
   }

   public Optimizer getOptimizer (RegistrationTransformer transformer) {
      if (! getTransformerList().contains (transformer)) {
         throw new IllegalArgumentException ("Master transformer not "
         + "in the registration action list");
      }
      return myActions.get (transformer);
   }

   public Renderable [] getTransformerRenderers () {
      RegistrationTransformer [] rts = getRenderableTransformers ();
      Renderable [] rmb = new Renderable [rts.length]; 
      for (int i = 0; i < rmb.length; i++) {
         HasRenderableModel hasrender = (HasRenderableModel) rts[i];
         rmb[i] = hasrender.getTransformerRenderer ();
      }
      return rmb;
   }

   public RegistrationTransformer [] getRenderableTransformers () {
      ArrayList<RegistrationTransformer> list = new ArrayList<RegistrationTransformer>();
      for (RegistrationTransformer tfm : getTransformers ()) {
         if (HasRenderableModel.class.isAssignableFrom (tfm.getClass ())) {
            list.add (tfm);
         }
      }
      RegistrationTransformer [] rts = new RegistrationTransformer [list.size ()];
      for (int i = 0; i < rts.length; i++) {
         rts[i] = (RegistrationTransformer) list.get (i);
      }
      return rts;
   }

   public RegistrationTransformer [] getMasterTransformers () {
      ArrayList<RegistrationTransformer> list = new ArrayList<RegistrationTransformer>();
      for (RegistrationTransformer tfm : getTransformers ()) {
         if (HasSlaves.class.isAssignableFrom (tfm.getClass ())) {
            list.add (tfm);
         }
      }
      RegistrationTransformer [] rts = new RegistrationTransformer [list.size ()];
      for (int i = 0; i < rts.length; i++) {
         rts[i] = (RegistrationTransformer) list.get (i);
      }
      return rts;
   }

   public TransformableGeometry [] getSlaveGeometries (HasSlaves master) {
      if (! getTransformerList().contains (master)) {
         throw new NullPointerException ("Master transformer not "
         + "in the registration action list");
      }
      Object [] objs = master.getSlaves ().toArray ();
      TransformableGeometry [] slaves = new TransformableGeometry [objs.length];
      int  idx = 0;
      for (Object obj : objs) {
         slaves[idx++] = (TransformableGeometry) obj;
      }
      return slaves;
   }

   // -------------------------------------
   // implement rendering
   // -------------------------------------

   @Override
   public void prerender (RenderList list) {
      if (actionVisible) {
         for (RegistrationTransformer tfm : getTransformers ()) {
            if (tfm instanceof HasRenderableModel) {
               HasRenderableModel hasrender = (HasRenderableModel) tfm;
               list.addIfVisible (hasrender.getTransformerRenderer ());
               hasrender.getTransformerRenderer ().prerender (list);
            }
         }
      }
      super.prerender (list);
   }



   @Override
   public StepAdjustment advance (double t0, double t1, int flags) {
      if (iterative) {
         if (!oldIterative) {
            //myMap.update ();
            //for (SlaveInfo slave : mySlaves.values ()) {
               //slave.update ();
            //}
            oldIterative = true;
         }
         takeIterativeActions ();
      } else {
         oldIterative = false;
      }
      return null;
   }

   // -------------------------------------
   // implement action listener
   // -------------------------------------


   @Override
   public void actionPerformed(ActionEvent event) {

      RegistrationTransformer [] transformers = getTransformers ();

      for (int i = 0; i < menuItemNames.length; i++) {
         if (event.getActionCommand().equals (menuItemNames[i])) {
            resetTime();
            if (menuItemNames[i].startsWith ("reset action")) {
               resetAction (transformers[i/2], null);
            }
         }
      }

      transformers = getMasterTransformers ();
      for (int i = 0; i < menuSlaveActionItemNames.size (); i++) {
         if (event.getActionCommand ().equals (menuSlaveActionItemNames.get (i))) {
            resetTime();
            HasSlaves hs = (HasSlaves) transformers[i/2];
            if (menuItemNames[i].startsWith ("reset action")) {
               for (TransformableGeometry geometry : hs.getSlaves ()) {
                  restoreTransform (transformers[i/2], geometry);
               }
            }
         }
      }

      if (event.getSource ().equals (updateMapButton)) {
         try {
            myMap.update ();
            System.out.println ("Map updated!");
         }
         catch (Exception e) {
            e.printStackTrace ();
            System.err.println ("Failed to update map!");
         }
         
      }
      
      if (event.getSource ().equals (updateButton)) {
         try {
            for (RegistrationTransformer tf : myActions.keySet ()) {
               tf.update ();
            }
            System.out.println ("Transformers updated!");
         }
         catch (Exception e) {
            e.printStackTrace ();
            System.err.println ("Failed to update transformer!");
         }
         
         try {
            myMap.update ();
            System.out.println ("Map updated!");
         }
         catch (Exception e) {
            e.printStackTrace ();
            System.err.println ("Failed to update map!");
         }
      }
   }

   private void resetTime() {
      RootModel root = RootModel.getRoot (this);
      if (root != null) {
         root.getWayPoint(0).setValid (false);
         Main main = Main.getMain();
         if (main != null) {
            main.reset();
         }
      }
   }

   //TODO : need to be fixed
   @Override
   public boolean getMenuItems(List<Object> items) {

      RegistrationTransformer [] transformers = getTransformers ();
      int N = transformers.length;
      menuItemNames = new String [2*N];

      JMenuItem menuItem;
      String name;
      for (int i = 0; i < N; i++) {
         name = "action" + Integer.toString (i) + "[ ";
         name = name + transformers[i].getClass ().getSimpleName ();
         name = name + ", ";
         name = name + myActions.get (transformers[i]).getClass ().getSimpleName ();
         name = name + " ]";
         menuItemNames[2*i] = new String (name);
         menuItem = makeMenuItem (name, "take registration action" + 
         Integer.toString (i));
         items.add (menuItem);

         name = "reset " + name;
         menuItem = makeMenuItem (name, "reset registration action" + 
         Integer.toString (i));
         items.add (menuItem);
         menuItemNames[i*2+1] = new String (name);

      }

      menuSlaveActionItemNames.clear ();
      for (int i = 0; i < N; i++) {
         if (HasSlaves.class.isAssignableFrom (transformers[i].getClass ())){
            name = "apply action";
            name = name + i + " on slaves";
            menuSlaveActionItemNames.add (new String(name));
            menuItem = makeMenuItem (name, "apply action" + 
            Integer.toString (i) + "on its slaves");
            items.add (menuItem);
            name = "reset action";
            name = name + i + " on slaves";
            menuSlaveActionItemNames.add (new String(name));
            menuItem = makeMenuItem (name, "reset action" + 
            Integer.toString (i) + "on its slaves");
            items.add (menuItem);
         }
      }

      return true;
   }   

   protected JMenuItem makeMenuItem (String cmd, String toolTip) {
      JMenuItem item = new JMenuItem(cmd);
      item.addActionListener(this);
      item.setActionCommand(cmd);
      if (toolTip != null && !toolTip.equals ("")) {
         item.setToolTipText (toolTip);
      }
      return item;
   }

   public boolean hierarchyContainsReferences() {
      return false;
   }

   private void mainPause() {
      RootModel root = RootModel.getRoot (this);
      if (root != null) {
         root.getWayPoint(0).setValid (false);
         Main main = Main.getMain();
         if (main != null) {
            main.pause ();
         }
      }
   }

   @Override
   public void render (Renderer renderer, int flags) {
      // TODO Auto-generated method stub

   }









}
