package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public interface SparseQuadLSFormulator 
extends QuadLSFormulator{

   public void addQuadraticTerm (SparseBlockMatrix H);

   public void addPropotionalTerm (VectorNd q);

   /**
    * This term has no influence on optimization, but for 
    * estimating energy value;
    * @param c
    */
   public void addConstantTerm (double c);

}
