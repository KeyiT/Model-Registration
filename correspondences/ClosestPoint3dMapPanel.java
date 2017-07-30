package artisynth.models.swallowingRegistrationTool.correspondences;

import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.workspace.RootModel;
import maspack.properties.PropertyList;

public class ClosestPoint3dMapPanel extends ControlPanel{
   
 //--------------------------implements properties-----------------------//
   
   protected boolean srcToTgtEnabled = true;
   protected boolean tgtToSrcEnabled = true;

   public static PropertyList myProps =
   new PropertyList (ClosestPoint3dMapPanel.class, ControlPanel.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.add ("srcToTgtMapping", "source to target mapping", true);
      myProps.add ("tgtToSrcMapping", "target to source mapping", true);
   }

   public void setSrcToTgtMapping(boolean dual) {
      srcToTgtEnabled = dual;
   }

   public boolean getSrcToTgtMapping() {
      return srcToTgtEnabled;
   }

   public void setTgtToSrcMapping(boolean dual) {
      tgtToSrcEnabled = dual;
   }

   public boolean getTgtToSrcMapping() {
      return tgtToSrcEnabled;
   }
   
   private void addWidgetsToExternalPanel (ControlPanel panel) {
      if (panel.numWidgets () != 0) panel.addWidget (new JSeparator ());
      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");
   }

   public void addWidgetsToExternalPanel (ControlPanel panel, RootModel root) {
      addWidgetsToExternalPanel (panel);
      root.add (this);
      setVisible (false);
   }

}
