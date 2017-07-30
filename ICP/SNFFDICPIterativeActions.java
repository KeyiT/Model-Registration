package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.CompositeIterativeActionsBase;
import artisynth.models.swallowingRegistrationTool.IterativeTask;
import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMapPanel;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.ImproperStateException;
import maspack.matrix.VectorNd;

public class SNFFDICPIterativeActions extends CompositeIterativeActionsBase{

   NFFDeformer myFFD;
   Set<SlaveInfo> myActiveSlaves = new HashSet<SlaveInfo> ();

   private VectorNd oldFFDP = new VectorNd ();
   
   private SNFFDICPTask myTask;

   @Override
   protected IterativeTask createIterativeTask () {
      myTask = new SNFFDICPTask ();
      
      NFFDPanel panel = myPanels.get (myFFD);
      if (panel != null) {
         try {
            myTask.setNFFDPanel (panel.CloneForData ());
         }
         catch (CloneNotSupportedException e) {
            e.printStackTrace();
            System.err.println (
               "Failed to clone NFFD panel data!");
         }
      }
      
      return myTask;
   }

   @Override
   protected RegistrationTransformer globalPickAction (
      List<RegistrationTransformer> tfs) {

      myFFD = null;
      for (RegistrationTransformer tf : tfs) {
         if (tf instanceof NFFDeformer) {
            myFFD = (NFFDeformer)tf;
            return tf;
         }
      }
      System.err.println ("NFFDeformer not found!");
      return null;
   }

   @Override
   protected boolean globalAddSlave (Collection<TransformableGeometry> slaves) {
      return false;
   }

   @Override
   protected void globalPickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves) {
      activeSlaves.addAll (slaves);
      myActiveSlaves.clear ();
      myActiveSlaves.addAll (slaves);
   }

   @Override
   protected boolean globalPreaction (RegistrationManager manager) {
      // control map directions
      if (myMapPanel != null) {
         ClosestPoint3dMap map = (ClosestPoint3dMap) 
         manager.getCloudMap ();

         map.enableSourceToTargetMapping (
            myMapPanel.getSrcToTgtMapping ());
         map.enableTargetToSourceMapping (
            myMapPanel.getTgtToSrcMapping ());
      }
      
      myFFD.getCoef (oldFFDP);

      return true;
   }

   @Override
   protected boolean globalPostUpdate (RegistrationManager manager) {
      
      //if (! myTask.isFFDNeedUpgrade ()) {
         // need to be fixed
         myFFD.upgrade ();
         myFFD.defineFFDForSlaves ();
         myFFD.updateSlaveInfos ();
         System.out.println ("Upgrade!");
         return true;
      //}

      // measure energy decreasing
         /*
      VectorNd p = new VectorNd ();
      myFFD.getCoef (p);
      p.sub (oldFFDP);*/

      //return true;
   }


   //-------------------------panel----------------------------//

   HashMap <NFFDeformer, NFFDPanel> myPanels = 
   new HashMap <NFFDeformer, NFFDPanel>(); 

   ClosestPoint3dMapPanel myMapPanel;

   public static NFFDPanel createControlPanel (NFFDeformer ffd) {
      if (ffd == null) {
         throw new ImproperStateException (
         "FFDeformer not initialzed");
      }
      NFFDPanel panel = new NFFDPanel (ffd);
      return panel;
   }

   public NFFDPanel getControlPanel (NFFDeformer ffd) {
      return myPanels.get (ffd);
   }

   public NFFDPanel addControlPanel (NFFDPanel panel) {
      return myPanels.put (panel.getFFD (), panel);
   }

   public void controlFFD (NFFDeformer ffd, 
      ControlPanel hostPanel, RootModel root) {
      NFFDPanel panel = createControlPanel (ffd);
      panel.createSlavePanel ();
      addControlPanel (panel);
      panel.addWidgetsToExternalPanel (hostPanel, root);
   }

   public void controlSlave (
      NFFDSlaveInfo info, ControlPanel panel, RootModel root) {
      NFFDeformer ffd = info.getTransformer ();
      NFFDPanel ffdP = myPanels.get (ffd);
      if (ffdP == null) {
         throw new ImproperStateException (
         "NFFDPanel for this transformer not initilized!");
      }

      ffdP.controlSlave (info, panel, root);
   }

   public void controlMapDirection(ControlPanel hostPanel, RootModel root) {
      myMapPanel = new ClosestPoint3dMapPanel ();
      myMapPanel.addWidgetsToExternalPanel (hostPanel, root);
   }

}
