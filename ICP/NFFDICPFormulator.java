package artisynth.models.swallowingRegistrationTool.ICP;

import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;

public class NFFDICPFormulator extends BLDICPFormulator{
   
   protected NFFDeformer myDeformer;
   
   @Override
   public void setTransformer (RegistrationTransformer tf) {
      if (tf instanceof NFFDeformer) {
         myDeformer = (NFFDeformer)tf;
         super.setTransformer (tf);
         return;
      }

      throw new ClassCastException ("Incompatible transformer!");
   }
   
   @Override
   public NFFDeformer getTransformer () {
      return myDeformer;
   }

}
