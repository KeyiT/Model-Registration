package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.swallowingRegistrationTool.utilities.PseudoInverse;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

public class SVDPseudoInverseOptimizer extends LinearOptimizer{

   PseudoInverse  pInv = new PseudoInverse();

   @Override
   protected Vector findSolution (LinearOptimizerFormulator form) {
      MatrixNd A = new MatrixNd ();
      VectorNd b = new VectorNd ();
      
      if (form.getCostFunction (A, b)) {
         return null;
      }

      if (A.rowSize () != b.size ()) {
         throw new ImproperSizeException ("Incompatible size!");
         
      }
      
      VectorNd output = new VectorNd ();
      
      try {
      pInv.set (A);
      pInv.pseInvert ();
      output.mul (pInv.getInv (), b); 
      }
      catch (Exception e) {
         System.err.println ("Failed to do pseudo-inverse!");
         return null;
      }
      
      return output; 
   }
}
