package artisynth.models.swallowingRegistrationTool.ICP;


import java.util.List;

import artisynth.models.swallowingRegistrationTool.main.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.transformers.ARAPDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.MeshBasedLinearDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public class ARAPICPIterativeActions extends ICPMLDIterativeActions{
   
   ARAPDeformer myARAP;
   
   @Override
   public MeshBasedLinearDeformer pickAction (
      List<RegistrationTransformer> tfs) {
      for (RegistrationTransformer tf : tfs) {
         if (tf instanceof ARAPDeformer) {
            myDeformer = (ARAPDeformer)tf;
            myARAP = (ARAPDeformer)tf;
            break;
         }
      }
      
      return myDeformer;
   }

   @Override
   public boolean preAction (RegistrationManager manager) {
      myARAP.advanceRigidModes ();
      reduceRigidty (manager);
      return super.preAction (manager);
   }
   
   
   double oldErr = Double.MAX_VALUE;
   double oldaw = getRigidity ();
   int step = 300;
   
   //TODO: add to panel
   protected boolean reduceRigidty (RegistrationManager manager) {
      double minRigidity = super.getRigidity () * 0.02;
      
      double err =  manager.getRegErr ();
      double red = oldErr - err;
      double ratio =  red / oldErr;
      oldErr = err;
      
      double raw = getActualRigidity ();
      System.out.println (raw);
      if (oldaw != getRigidity()) {
         raw = getRigidity ();
         oldaw = getRigidity ();
         setActualRigidity (raw);
      }
      else if (ratio < 0.1 && ratio > -0.05 && raw > minRigidity) {
         raw = raw - (getRigidity () - minRigidity) / (double)step;
         if (raw > minRigidity) {
            setActualRigidity (raw);
            return true;
         }
      }
      else if (ratio < -0.05) {
         System.err.println ("warning: Err increasing! Reset rigidity!");
         setActualRigidity (getRigidity());
      }
      
      return false;
   }
}
