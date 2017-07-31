package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

import artisynth.models.swallowingRegistrationTool.main.IterativeTask;
import artisynth.models.swallowingRegistrationTool.main.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo.SlaveType;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDSlavePanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.VectorNd;

public class SNFFDICPTask implements IterativeTask{

   NFFDeformer myFFD;
   SNFFDICPFormulator myForm;
   NFFDFormulator myFFDForm;   
   Set<SlaveInfo> myActiveSlaves = new HashSet<SlaveInfo> ();

   int myTotalIter;
   int myTaskIter;

   NFFDPanel myNFFDPanel;
   
   VectorNd oldP = new VectorNd ();

   private boolean needUpgrade = false;
   double regularIncrease = 0.0;

   public void setNFFDPanel (NFFDPanel panel) {
      myNFFDPanel = panel;
   }

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      if (tf instanceof NFFDeformer) {
         myFFD = (NFFDeformer) tf;
         myForm =  new SNFFDICPFormulator ();
         myFFDForm = new NFFDFormulator (myFFD);

         if (myFFD.getCtrlPntsNum () > 700) {
            myFFD.enableSubdivision (true);
            myTotalIter = myFFD.makeSubdomains ();
         }
         else {
            myFFD.enableSubdivision (false);
            myTotalIter = 1;
         }
         
         myFFD.getCoef (oldP);

         return;
      }
      throw new UnsupportedOperationException (
         "Incompatible transformer");
   }
   

   @Override
   public void setActiveSlaves (Collection<SlaveInfo> activeSlaves) {
      myActiveSlaves.clear ();
      myActiveSlaves.addAll (activeSlaves);
   }
   
   @Override
   public void setIteration (int actionIteration, int taskIteration) {
      if (taskIteration == 0) {
         myTaskIter = 0;
      }

      System.out.println ("Action Iteration: " + actionIteration);
      System.out.println ("Task Iteration: " + taskIteration);
      System.out.println ("Real task Iteration: " + myTaskIter);
   }

   @Override
   public boolean preAction (RegistrationManager manager) {

      myForm.clearExternalTerm ();
      myForm.clearExternalGoal ();


      if (myFFD.isSubdivisionEnabled ()) {
         myFFD.setWorkingSubdomain (myTaskIter);
         myForm.setDimension (myFFD.numSubdomainCtrlPnts ()*3);
         formulateForSubdomain ();
      }
      else {
         // no subdivision
         myForm.setDimension (myFFD.getCtrlPntsNum ()*3);
         formulate ();
      }

      return true;
   }

  

   @Override
   public Formulator getFormulator () {
      if (myTaskIter == 0) {
         myForm.validateSubdomainData (false);
      }
      else {
         myForm.validateSubdomainData (false);
      }
      return myForm;
   }

   /**
    * {@inheritDoc}
    * 
    * If the value of <tt>optimized</tt> is false, reset and increase
    * regularization weight and do it again;
    */
   @Override
   public boolean postOptimize (
      RegistrationManager manager, boolean optimized) {
      
      if (! optimized) {
         if (myFFD.isSubdivisionEnabled ()) {
            myFFD.setCoef (oldP);
            myFFD.getFFD ().updateMeshes ();
            myFFD.fastTransformPointBasedSlaves ();
            manager.getCloudMap ().update ();
            myTaskIter = 0;
            regularIncrease += 0.3;
         }
         else {
            myTaskIter ++;
            needUpgrade = true;
         }
         
         return false;
      }
      else {
         myTaskIter ++;
      }
      
      if (myTotalIter <= myTaskIter) {
         return true;
      }

      return true;
   }

   /**
    * {@inheritDoc}
    * 
    * If every sub-domain has been gone through
    * return false, otherwise return true;
    */
   @Override
   public boolean postUpdate (RegistrationManager manager) {
      if (myTotalIter <= myTaskIter) {
         regularIncrease = 0.5;
         return false;
      }
    
      return true;
   }

   public boolean isFFDNeedUpgrade () {
      return needUpgrade;
   }

   
   private void formulateForSubdomain () {
      if (myNFFDPanel != null) {
         
         // enforce this term to guanrrentee bijective  tranform
         myFFDForm.regularizeSubdomainGridByDisplace (
            myForm, myNFFDPanel.getCtlPntWeight () + regularIncrease);
         
         if (myNFFDPanel.getEdgeWeight () != 0) {
            myFFDForm.regularizeSubdomainGridByEdge (myForm, myNFFDPanel.getEdgeWeight ());
         }
         
         for (SlaveInfo info: myActiveSlaves) {
            NFFDSlavePanel slaveP = myNFFDPanel.getSlavePanel (info);
            NFFDSlaveInfo slaveInfo = (NFFDSlaveInfo)info.getTransformerInfo (myFFD);
            if (info.getSlaveType () == SlaveType.Fem) {
               if (slaveP.getStrainWeight () != 0) {
                  myFFDForm.regularizeSubdomainSlaveStrain (myForm, null, null, 
                     slaveInfo, slaveP.getStrainWeight ());
               }
            }
         }
      }
      else {
         myFFDForm.regularizeSubdomainGridByDisplace (myForm, 1.0 + regularIncrease);
      }
   }


   private void formulate () {
      if (myNFFDPanel != null) {
         if (myNFFDPanel.getCtlPntWeight () != 0) {
            myFFDForm.regularizeGridByDisplace (myForm, myNFFDPanel.getCtlPntWeight ());
         }
         if (myNFFDPanel.getEdgeWeight () != 0) {
            myFFDForm.regularizeGridByEdge (myForm, myNFFDPanel.getEdgeWeight ());
         }

         for (SlaveInfo info: myActiveSlaves) {
            NFFDSlavePanel slaveP = myNFFDPanel.getSlavePanel (info);
            NFFDSlaveInfo slaveInfo = (NFFDSlaveInfo)info.getTransformerInfo (myFFD);
            if (info.getSlaveType () == SlaveType.Fem) {
               if (slaveP.getStrainWeight () != 0) {
                  myFFDForm.regularizeWarpedSlaveStrain (myForm, null, null, 
                     slaveInfo, slaveP.getStrainWeight ());
               }
               if (slaveP.getQualityWeight () != 0) {
                  myFFDForm.regularizeFEMSlaveMeshQuality1 (myForm, null, null, 
                     slaveInfo, slaveP.getQualityWeight ());
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
   }

}
