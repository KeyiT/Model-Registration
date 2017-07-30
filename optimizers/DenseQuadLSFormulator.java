package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public interface DenseQuadLSFormulator extends QuadLSFormulator{
   
   
   public void addQuadraticTerm (MatrixNd H);
   
   public void addPropotionalTerm (VectorNd q);
   
   /**
    * This term has no influence on optimization, but for 
    * estimating energy value;
    * @param c
    */
   public void addConstantTerm (double c);
}
