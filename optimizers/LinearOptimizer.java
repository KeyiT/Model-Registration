package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

/**
 * Given a matrix A and vector b find a vector x minimize |Ax - b|
 * @author KeyiTang
 *
 */
public abstract class LinearOptimizer extends Optimizer{

   @Override
   public boolean checkSink(OptimizerSink tf) {
      if (tf instanceof LinearOptimizerSink) {
         return true;
      }
      return false;
   }
   
   @Override
   public boolean checkFormulator (Formulator F) {
      if (F instanceof LinearOptimizerFormulator) {
         return true;
      }
      return false;
   }
   
   @Override
   public final boolean optimize(
      OptimizerSink sink, Formulator form) {
      if (!checkSink (sink)) {
         throw new IllegalArgumentException ("Incompatible sink!");
      }
      if (!checkFormulator (form)) {
         throw new IllegalArgumentException ("Incompatible formulator!");
      }
      
      Vector output = findSolution ((LinearOptimizerFormulator)form);
      
      if (output == null) {
         return false;
      }
      LinearOptimizerSink lsink = (LinearOptimizerSink) sink;
      lsink.takeOutput (output);
      return true;
   }
   
   /**
    * Given a matrix A and vector b find a vector x minimize |Ax - b|
    * 
    * @param form <code>LinearOptimizerFormulator<code>
    * @return solution
    * <p>
    * If success to find the solution return it otherwise return <code>Null</code>
    */
   protected abstract Vector findSolution (LinearOptimizerFormulator form);
   
}
