package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.List;

import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.transformers.ACAPDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.MeshBasedLinearDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public class ACAPICAIterativeActions extends ICPMLDIterativeActions{
   
   ACAPDeformer myACAP;
   
   
   
   @Override
   public MeshBasedLinearDeformer pickAction (
      List<RegistrationTransformer> tfs) {
      for (RegistrationTransformer tf : tfs) {
         if (tf instanceof ACAPDeformer) {
            myDeformer = (ACAPDeformer)tf;
            myACAP = (ACAPDeformer)tf;
            break;
         }
      }
      
      return myDeformer;
   }

   @Override
   public boolean preAction (RegistrationManager manager) {
      //myACAP.advanceRigidModes ();
      reduceRigidty (manager);
      return super.preAction (manager);
   }
   
   double oldErr = Double.MAX_VALUE;
   double oldaw = getRigidity ();
   int step = 100;
   
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
