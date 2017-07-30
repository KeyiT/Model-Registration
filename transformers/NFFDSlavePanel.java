package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.event.ActionEvent;

import javax.swing.JButton;
import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo.SlaveType;
import artisynth.models.swallowingRegistrationTool.utilities.DataClonable;
import maspack.properties.PropertyList;

public class NFFDSlavePanel extends ControlPanel implements DataClonable{

   NFFDSlaveInfo mySlaveInfo;
   SlaveInfo myInfo;
   
   JButton advanceButton = new JButton ("Advance Undeformed State");
   
   public NFFDSlavePanel (NFFDSlaveInfo info) {
      mySlaveInfo = info;
      myInfo = info.getSlaveInfo ();
      addWidgetsToExternalPanel (this);
      advanceButton.addActionListener (this);
   }
   
   public NFFDSlaveInfo getSlaveInfo () {
      return mySlaveInfo;
   }
   
   private void addWidgetsToExternalPanel (ControlPanel panel) {
      if (myInfo.hasEdge ()) {
         //if (panel.numWidgets () != 0) panel.addWidget (new JSeparator ());
         panel.addLabel ("Slave" + " Regularization Weights");
         if (myInfo.getSlaveType () == SlaveType.Fem) {
            panel.addWidget ("control FEM Strain", this, "strainWeight");
            panel.addWidget ("control FEM Quality", this, "qualityWeight");
         }
         else if (myInfo.hasMeanCurvatureNormal ()) {
            panel.addWidget ("control As-conformal-as-possible energy", this, "ACAPWeight");
            panel.addWidget ("control As-rigid-as-possible energy", this, "ARAPWeight");
            panel.addWidget ("control bending energy", this, "bendingWeight");
            panel.addWidget ("control Laplacian-smooth energy", this, "laplacianWeight"); 
         }
         else {
            panel.addWidget ("control Edge deformation", this, "edgeWeight");
         }
      }
      if (myInfo.hasPoint ()) {
         panel.addWidget (advanceButton);
      }
   }

   public void addWidgetsToExternalPanel (ControlPanel panel, RootModel root) {
      addWidgetsToExternalPanel (panel);
      
      root.add (this);
      setVisible (false);
   }
   
   //--------------------------implements properties-----------------------//

   public static PropertyList myProps =
   new PropertyList (NFFDSlavePanel.class, ControlPanel.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.add ("strainWeight", "weight for regularizing FEM strain", 1.0);
      myProps.add ("qualityWeight", "weight for regularizing FEM quality", 0.0);
      myProps.add ("edgeWeight", "weight for regularizing slave edges", 1.0);
      myProps.add ("laplacianWeight", "weight for slave laplacian "
      + "smooth energy", 0.0);
      myProps.add ("ARAPWeight", "weight for regularizing slave following  "
      + "As-Rigid-As-Possible strategy", 0.0);
      myProps.add ("ACAPWeight", "weight for regularizing slave following  "
      + "As-Conformal-As-Possible strategy", 0.0);
      myProps.add ("bendingWeight", "weight for regularizing slave bending  "
      + "energy", 0.0);
   }

   private double sw = 0.0;
   private double qw = 0.0;
   private double ew = 0.0;
   private double lw = 0.0;
   private double aw = 0.0;
   private double bw = 0.0;
   private double cw = 0.0;

   public void setStrainWeight (double w) {
      sw = w;
   }

   public double getStrainWeight () {
      return sw;
   }
   
   public void setQualityWeight (double w) {
      qw = w;
   }

   public double getQualityWeight () {
      return qw;
   }

   public void setEdgeWeight (double w) {
      ew = w;
   }

   public double getEdgeWeight () {
      return ew;
   }
   
   public void setLaplacianWeight (double w) {
      lw = w;
   }

   public double getLaplacianWeight () {
      return lw;
   }
   
   public void setARAPWeight (double w) {
      aw = w;
   }
   
   public double getARAPWeight () {
      return aw;
   }
   
   public void setBendingWeight (double w) {
      bw = w;
   }

   public double getBendingWeight () {
      return bw;
   }
   
   public void setACAPWeight (double w) {
      cw = w;
   }
   
   public double getACAPWeight () {
      return cw;
   }
   
   @Override
   public void actionPerformed(ActionEvent event) {
      if (event.getSource().equals (advanceButton)) {
         myInfo.savePointCurretAsUndeformed ();
      }
   }

   @Override
   public NFFDSlavePanel CloneForData () throws CloneNotSupportedException {
      NFFDSlavePanel panel = new NFFDSlavePanel (mySlaveInfo);
      panel.setStrainWeight (sw);
      panel.setQualityWeight (qw);
      panel.setEdgeWeight (ew);
      panel.setLaplacianWeight (lw);
      panel.setARAPWeight (aw);
      panel.setBendingWeight (bw);
      return panel;
   }

}
