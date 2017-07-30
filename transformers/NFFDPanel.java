package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.event.ActionEvent;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.utilities.DataClonable;
import maspack.properties.PropertyList;

public class NFFDPanel extends ControlPanel implements DataClonable{

   NFFDeformer myFFD;
   
   private boolean slavePanelP = true;
   
   private HashMap<SlaveInfo, NFFDSlavePanel> mySlavePanels = 
   new HashMap<SlaveInfo, NFFDSlavePanel> ();
   
   JButton advanceSlavesButton = new JButton ("Advance Slaves Undeformed States");

   public NFFDPanel (NFFDeformer myFFD) {
      setFFD (myFFD);
      addWidgetsToExternalPanel (this);
      advanceSlavesButton.addActionListener (this);
   }

   private void setFFD (NFFDeformer ffd) {
      myFFD = ffd;
   }

   public NFFDeformer getFFD () {
      return myFFD;
   }
   
   public void enableSlavePanel (boolean enable) {
      slavePanelP = enable;
   }
   
   public boolean isSlavePanelEnabled () {
      return slavePanelP;
   }
   
   public NFFDSlavePanel getSlavePanel (SlaveInfo info) {
      return mySlavePanels.get (info);
   }

   public void getSlavePanels (Map<SlaveInfo, NFFDSlavePanel> map) {
      map.putAll (mySlavePanels);
   }
   
   private void addWidgetsToExternalPanel (ControlPanel panel) {
      if (panel.numWidgets () != 0) panel.addWidget (new JSeparator ());
      panel.addLabel ("NFFD");
      panel.addLabel ("Crid Regularization Weights");
      panel.addWidget ("control points displacement", this, "ctlPntWeight");
      panel.addWidget ("Edge deformation", this, "edgeWeight");
      panel.addWidget ("control grid strain", this, "strainWeight");
      panel.addWidget (advanceSlavesButton);
   }
   
   public void createSlavePanel () {
      List<SlaveInfo> infos = new ArrayList<SlaveInfo> ();
      myFFD.getSlaveInfos (infos);
      for (SlaveInfo info : infos) {
         NFFDSlavePanel slavePanel = new NFFDSlavePanel (
            (NFFDSlaveInfo)info.getTransformerInfo (myFFD));
         mySlavePanels.put (info, slavePanel);
      }
   }

   public void addWidgetsToExternalPanel (ControlPanel panel, RootModel root) {
      addWidgetsToExternalPanel (panel);

      if (slavePanelP) {
         List<SlaveInfo> infos = new ArrayList<SlaveInfo> ();
         myFFD.getSlaveInfos (infos);
         for (SlaveInfo info : infos) {
            NFFDSlavePanel slavePanel = mySlavePanels.get (info);
            slavePanel.addWidgetsToExternalPanel (panel, root);
         }
      }
      
      root.add (this);
      setVisible (false);
   }
   
   public void controlSlave (NFFDSlaveInfo info, 
      ControlPanel panel, RootModel root) {
      if (! (info.getTransformer () == myFFD)) {
         throw new IllegalArgumentException (
            "Not my slave!");
      }
      
      NFFDSlavePanel np = new NFFDSlavePanel (info);
      np.addWidgetsToExternalPanel (panel, root);
      mySlavePanels.put (info.getSlaveInfo (), np);
   }

   //--------------------------implements properties-----------------------//

   public static PropertyList myProps =
   new PropertyList (NFFDPanel.class, ControlPanel.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.add ("ctlPntWeight", "weight for regularizing control "
      + "points displacement", 0);
      myProps.add ("edgeWeight", "weight for regularizing the change "
      + "of grid edges", 1.0);
      myProps.add ("strainWeight", "weight for regularizing the grid "
      + "strain energy", 0.0);
   }

   // weight for grid nodes displacement
   private double pdw = 0.0;
   // weight for grid edge change
   private double ecw = 1.0;
   // weight for grid strain
   private double sw = 0.0;

   public void setCtlPntWeight (double w) {
      pdw = w;
   }

   public double getCtlPntWeight () {
      return pdw;
   }

   public void setEdgeWeight (double w) {
      ecw = w;
   }

   public double getEdgeWeight () {
      return ecw;
   }
   
   public void setStrainWeight (double w) {
      sw = w;
   }

   public double getStrainWeight () {
      return sw;
   }

   @Override
   public void actionPerformed(ActionEvent event) {
      if (event.getSource().equals (advanceSlavesButton)) {
         myFFD.advanceSlaveUndeformedState ();
      }
   }
   
   

   @Override
   public NFFDPanel CloneForData () throws CloneNotSupportedException {
      NFFDPanel panel = new NFFDPanel (myFFD);
      panel.setCtlPntWeight (pdw);
      panel.setEdgeWeight (ecw);
      panel.setStrainWeight (sw);
      panel.enableSlavePanel (slavePanelP);
      
      for (SlaveInfo info : mySlavePanels.keySet ()) {
         NFFDSlavePanel spanel = mySlavePanels.get (info).CloneForData ();
         panel.mySlavePanels.put (info, spanel);
      }
      
      return panel;
   }

}
