package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;
import maspack.solvers.PardisoSolver;

/**
 * A naive implementation of quadratic optimization
 * @author KeyiTang
 *
 */
public class SparseQuadraticLinearOptimizer extends LinearOptimizer{
   
   @Override
   public boolean checkFormulator (Formulator form) {
      if (form instanceof QuadLSFormulator) {
         return true;
      }
      return false;
   }
   

   @Override
   protected Vector findSolution (LinearOptimizerFormulator form) {
      
      SparseBlockMatrix H = new SparseBlockMatrix ();
      VectorNd q = new VectorNd ();
      
      checkFormulator (form);
      QuadLSFormulator QForm = (QuadLSFormulator) form;
      
      try {
         QForm.getQuadraticCostFunction (H, q);
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println ("Failed to get cost function!");
         return null;
      }
      
      if (!checkSize (H, q)) {
         return null;
      }
      
      q.scale (-0.5);
      VectorNd solution = new VectorNd ();
      solution.setSize (q.size ());
      
      try {
         PardisoSolver solver = new PardisoSolver ();
         solver.analyzeAndFactor (H);
         
         solver.solve (solution, q);
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println ("Solve failed!");
      }
      
      return solution;
   }
   
   private boolean checkSize (Matrix H, Vector q) {
      if (H.rowSize () != H.colSize ()) {
         throw new ImproperSizeException (
            "Hessian matrix must be a square matrix");
      }
      
      if (H.rowSize () != q.size ()) {
         throw new ImproperSizeException (
            "Incompatible proportional vector size!");
      }
      
      if (q.size () == 0) {
         System.out.println ("Warning: zero size quadratic "
         + "cost function!");
         return false;
      }
      
      return true;
   }

}
