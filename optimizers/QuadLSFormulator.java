package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.Matrix;
import maspack.matrix.Vector;

public interface QuadLSFormulator extends LinearOptimizerFormulator{
   /**
    * make quadratic cost function
    * @param H Hessian matrix 
    * @param q proportional term
    */
   public boolean getQuadraticCostFunction (Matrix H, Vector q);
   
   
}
