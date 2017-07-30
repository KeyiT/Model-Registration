package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.Matrix;

public interface GeneralizedPointInjectiveMapFormulator 
extends PointInjectiveMapFormulator{

   /**
    * Create weight matrix for source points;
    * @return 
    */
   public Matrix createSourceGeneralizeMatrix ();
   
   
   /**
    * Create weight matrix for target points;
    * @return 
    */
   public Matrix createTargetGeneralizeMatrix ();
}