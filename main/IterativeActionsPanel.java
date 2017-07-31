package artisynth.models.swallowingRegistrationTool.main;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.properties.PropertyList;

public class IterativeActionsPanel extends ControlPanel implements IterativeActions{

   ArrayList<IterativeActions> myActions = new ArrayList<IterativeActions> ();
   int myIdx = 0;

   // ------------------------- properties ------------------------------//

   public static PropertyList myProps =
   new PropertyList(IterativeActionsPanel.class, ControlPanel.class);

   static {
      myProps.add ("actionIndex", "pick iterative action", 0);
   }

   public PropertyList getAllPropertyInfo() {
      return myProps;
   }
   
   public void setActionIndex (int idx) {
      myIdx = idx;
   }
   
   public int getActionIndex () {
      return myIdx;
   }
   
   
   public IterativeActionsPanel () {
      addWidget ("Iterativa actions", this, "actionIndex");
   }
   
   public void addWidgetsToExternalPanel (ControlPanel panel, RootModel root) {
      if (panel.numWidgets () != 0) panel.addWidget (new JSeparator ());
      panel.addLabel ("Pick Iterative Actions");
      panel.addWidget ("Iterativa actions", this, "actionIndex");
      
      root.add (this);
      setVisible (false);
   }

   public void addIterativeActions (IterativeActions action) {
      if (!myActions.contains (action)) {
         myActions.add (action);
      }
   }

   public IterativeActions removeIterativeActions (int idx) {
      return myActions.remove (idx);
   }

   public IterativeActions getIterativeActions (int idx) {
      return myActions.get (idx);
   }
   
   public int numActions () {
      return myActions.size ();
   }

   @Override
   public RegistrationTransformer pickAction (
      List<RegistrationTransformer> tfs) {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return null;
      }

      return myActions.get (myIdx).pickAction (tfs);
   }

   @Override
   public boolean addSlave (Collection<TransformableGeometry> slaves) {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return false;
      }

      return myActions.get (myIdx).addSlave (slaves);
   }

   @Override
   public void pickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves) {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return;
      }

      myActions.get (myIdx).pickSlaves (activeSlaves, slaves);
   }

   @Override
   public Formulator getFormulator () {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return null;
      }
      return myActions.get (myIdx).getFormulator ();
   }

   @Override
   public boolean preAction (RegistrationManager manager) {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return false;
      }

      return myActions.get (myIdx).preAction (manager);
   }
   
   @Override
   public boolean postOptimize (
      RegistrationManager manager, boolean optimized) {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return false;
      }
      
      return myActions.get (myIdx).
      postOptimize (manager, optimized);
   }

   @Override
   public boolean postUpdate (RegistrationManager manager) {
      if (myIdx < 0 || myIdx >= myActions.size ()) {
         return false;
      }

      return myActions.get (myIdx).postUpdate (manager);
   }



}
