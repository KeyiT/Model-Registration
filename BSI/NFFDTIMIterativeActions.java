package artisynth.models.swallowingRegistrationTool.BSI;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.main.IterativeActions;
import artisynth.models.swallowingRegistrationTool.main.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMapPanel;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo.SlaveType;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDSlavePanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.ImproperStateException;
import maspack.matrix.VectorNd;

public class NFFDTIMIterativeActions implements 
IterativeActions{

   NFFDeformer myFFD;
   NFFDTIMFormulator myForm = new NFFDTIMFormulator ();
   NFFDFormulator myFFDForm = new NFFDFormulator();

   Set<SlaveInfo> myActiveSlaves = new HashSet<SlaveInfo> ();
   
   // minimum energy decreasing ratio 
   // if smaller than this value upgrade ffd
   private double upgradeRatio = 0.004;
   private boolean upgradeEnabled = true;
   
   /**
    * {@inheritDoc}
    * 
    * @param tfs
    * @return the first <code>NFFDeformer</code>
    */
   @Override
   public RegistrationTransformer pickAction (
      List<RegistrationTransformer> tfs) {

      myFFD = null;
      for (RegistrationTransformer tf : tfs) {
         if (tf instanceof NFFDeformer) {
            myFFD = (NFFDeformer)tf;
            myFFDForm.setTransformer (myFFD);
            myForm.setDimension (myFFD.getCtrlPntsNum ()*3);
            return tf;
         }
      }
      System.err.println ("NFFDeformer not found!");
      return null;
   }


   /**
    * {@inheritDoc}
    * 
    * No new slaves
    */
   @Override
   public boolean addSlave (Collection<TransformableGeometry> slaves) {
      return false;
   }

   /**
    * {@inheritDoc}
    * 
    * Include all slaves
    * 
    * @param activeSlaves
    * @param slaves
    */
   @Override
   public void pickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves) {
      activeSlaves.addAll (slaves);
      myActiveSlaves.clear ();
      myActiveSlaves.addAll (slaves);
   }

   @Override
   public Formulator getFormulator () {
      return myForm;
   }

   @Override
   public boolean preAction (RegistrationManager manager) {
      myForm.clearExternalTerm ();
      myForm.clearExternalGoal ();

   // control slaves
      NFFDPanel panel = myPanels.get (myFFD);
      if (panel != null) {
         if (panel.getCtlPntWeight () != 0) {
            myFFDForm.regularizeGridByDisplace (myForm, panel.getCtlPntWeight ());
         }
         if (panel.getEdgeWeight () != 0) {
            myFFDForm.regularizeGridByEdge (myForm, panel.getEdgeWeight ());
         }
         if (panel.getStrainWeight () != 0) {
            myFFDForm.regularizeGridByStrain (myForm, null, null, panel.getStrainWeight ());
         }

         for (SlaveInfo info: myActiveSlaves) {
            NFFDSlavePanel slaveP = panel.getSlavePanel (info);
            NFFDSlaveInfo slaveInfo = (NFFDSlaveInfo)info.getTransformerInfo (myFFD);
            if (info.getSlaveType () == SlaveType.Fem) {
               if (slaveP.getStrainWeight () != 0) {
                  myFFDForm.regularizeWarpedSlaveStrain (myForm, null, null, 
                     slaveInfo, slaveP.getStrainWeight ());
               }
               if (slaveP.getQualityWeight () != 0) {
                  myFFDForm.regularizeFEMSlaveMeshQuality (myForm, null, null, 
                     slaveInfo, slaveP.getQualityWeight ());
               }
            }
            else if (info.hasMeanCurvatureNormal ()) {
               if (slaveP.getACAPWeight () != 0) {
                  myFFDForm.regularizeSlaveMeshACAP (myForm, null, null, 
                     slaveInfo, slaveP.getACAPWeight ());
               }
               if (slaveP.getARAPWeight () != 0) {
                  myFFDForm.regularizeSlaveMeshARAP (myForm, null, null, 
                     slaveInfo, slaveP.getARAPWeight ());
               }
               if (slaveP.getBendingWeight () != 0) {
                  myFFDForm.regularizeSlaveMeshBending (myForm, null, null, 
                     slaveInfo, slaveP.getBendingWeight ());
               }
               if (slaveP.getLaplacianWeight () != 0) {
                  myFFDForm.regularizeSlaveMeshLaplacianSmooth (myForm, null, null, 
                     slaveInfo, slaveP.getLaplacianWeight ());
               }
               if (slaveP.getEdgeWeight () != 0) {
                  myFFDForm.regularizeSlaveEdge (myForm, null, null, 
                     slaveInfo, slaveP.getEdgeWeight ());
               }
            }
            else if (info.hasEdge ()) {
               if (slaveP.getEdgeWeight () != 0) {
                  myFFDForm.regularizeSlaveEdge (myForm, null, null, 
                     slaveInfo, slaveP.getEdgeWeight ());
               }
            }
         }
      }
      else {
         myFFDForm.regularizeGridByEdge (myForm, 1.0);
      }

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
   public boolean postOptimize (RegistrationManager manager, 
      boolean optimized) {
      return optimized;
   }

   VectorNd oldFFDP = new VectorNd ();
   int levelIterations = 0;
   boolean upgraded = false;

   @Override
   public boolean postUpdate (RegistrationManager manager) {
      
      upgraded = false;
      /*
      long freeM = Runtime.getRuntime ().totalMemory () - Runtime.getRuntime ().freeMemory ();
      double maxM = 8.8E9;
      
      
      double fdm  = (double)freeM / maxM;
      if (fdm > 0.98) {
         upgradeEnabled = false;
      }
      else {
         upgradeEnabled = true;
      }*/

      if (!manager.isOptimized () ) {
         // need to be fixed
         if (upgradeEnabled) {
            myFFD.upgrade ();
         }
         myFFD.defineFFDForSlaves ();
         myFFD.updateSlaveInfos ();
         System.out.println ("Upgrade!");
         
         levelIterations = 0;
         upgraded = true;
         return true;
      }

      // measure energy decreasing
      VectorNd p = new VectorNd ();
      myFFD.getCoef (p);
      p.sub (oldFFDP);
      double ratio = myForm.computeDecreasingRate (
         new VectorNd (p.size ()), p);

      // upgrade
      if (ratio <= upgradeRatio  || myFFD.hasLargeDistortion ()) {
         if (upgradeEnabled) {
            myFFD.upgrade ();
         }
         myFFD.defineFFDForSlaves ();
         myFFD.updateSlaveInfos ();
         System.out.println ("Upgrade!");
         
         upgraded = true;
         levelIterations = 0;
      }
      
      if (!upgraded) {
         levelIterations ++;
         if (levelIterations >= 10) {
            if (upgradeEnabled) {
               myFFD.upgrade ();
            }
            myFFD.defineFFDForSlaves ();
            myFFD.updateSlaveInfos ();
            System.out.println ("Upgrade!");
            
            upgraded = true;
            levelIterations = 0;
         }
      }
      
      return true;
   }
   
   public void setUpgradeRatio (double ratio) {
      upgradeRatio = ratio;
   }
   
   public double getUpgradeRatio () {
      return upgradeRatio;
   }
   
   public void resetUpgradeRatio () {
      upgradeRatio = 0.004;
   }
   

   public void enableUpgrade (boolean enable) {
      upgradeEnabled = enable;
   }
   
   public boolean isUpgradeEnabled () {
      return upgradeEnabled;
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
