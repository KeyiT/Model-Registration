package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.swallowingRegistrationTool.correspondences.CloudMap;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public interface Formulator {
   /**
    * This is for updating the registration transformer in each iteration
    * @param map
    */
   public void setTransformer (RegistrationTransformer tf);
   
   /**
    * This is for updating cloud map in each iteration
    * @param map
    */
   public void setCloudMap (CloudMap map);
}
