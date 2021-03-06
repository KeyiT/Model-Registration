package artisynth.models.swallowingRegistrationTool.ICP;

import artisynth.core.gui.ControlPanel;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.transformers.ACAPDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.MeshBasedLinearDeformer;
import maspack.properties.PropertyList;

public class ICPACAPManager extends ICPMLDManager{
   
   public static PropertyList myProps =
   new PropertyList (ICPACAPManager.class, ICPMLDManager.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.addReadOnly ("actualRigidity", "weights for rigidity energy term");
   }
   
   public double getActualRigidity () {
      return myMLDIAs.getActualRigidity ();
   }

   public ICPACAPManager (String name) {
     super (name);
   }

   @Override
   public MeshBasedLinearDeformer initializeDeformer () {
      return new ACAPDeformer ();
   }

   @Override
   public ICPMLDIterativeActions initializeMLDIAs () {
      return new ACAPICAIterativeActions ();
   }
   
   public ControlPanel createControlPanel (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");
      panel.addLabel ("Mesh Linear Deformer");
      panel.addWidget ("Initial Rigidity", this, "Rigidity");
      panel.addWidget ("Rigidity", this, "actualRigidity");

      myIAs.addWidgetsToExternalPanel (panel, root);
      createRegistrationErrorProbe (root);
      root.addControlPanel (panel);
      

      return panel;
   }

   public ControlPanel createControlPanelForMLDAction (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");
      panel.addLabel ("Mesh Linear Deformer");
      panel.addWidget ("Initial Rigidity", this, "Rigidity");
      panel.addWidget ("Rigidity", this, "actualRigidity");

      myIAs.setActionIndex (1);
      createRegistrationErrorProbe (root);
      root.addControlPanel (panel);
      

      return panel;
   }

}
