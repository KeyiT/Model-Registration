package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.Matrix;
import maspack.matrix.Vector;

public interface LinearOptimizerFormulator extends Formulator{
   
   /** Given a matrix A and vector b find a vector x minimize |Ax - b|
    * 
    * @param A return matrix 
    * @param b return vector
    * @return failed to form cost function return false, otherwise return
    * true
    */
   public boolean getCostFunction (Matrix A, Vector b);
   
   public void setDimension (int dimension);
   
   public int getDimension ();
}
