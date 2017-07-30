package artisynth.models.swallowingRegistrationTool.BSI;

import java.util.Collection;
import java.util.List;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.IterativeActions;
import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public class AffineTIMIterativeActions implements IterativeActions{
   
   TIMFormulator myForm = new TIMFormulator ();

   @Override
   public RegistrationTransformer pickAction (
      List<RegistrationTransformer> tfs) {
      for (RegistrationTransformer tf : tfs) {
         if (tf instanceof AffineModelTransformer) {
            return tf;
         }
      }
      System.err.println ("Affine transformer not found!");
      return null;
   }

   @Override
   public boolean addSlave (Collection<TransformableGeometry> slaves) {
      return false;
   }

   @Override
   public void pickSlaves (
      Collection<SlaveInfo> activeSlaves, Collection<SlaveInfo> slaves) {
      activeSlaves.addAll (slaves);
   }

   @Override
   public Formulator getFormulator () {
      return myForm;
   }

   @Override
   public boolean preAction (RegistrationManager manager) {
      return true;
   }
   
   @Override
   public boolean postOptimize (RegistrationManager manager, boolean optimized) {
      return optimized;
   }

   @Override
   public boolean postUpdate (RegistrationManager manager) {
      if (manager.getCloudMap ().getMappingError () <= 1E-6) {
         return false;
      }
      return true;
   }

}
