package artisynth.models.swallowingRegistrationTool.optimizers;


public abstract class Optimizer {

   /**
    * optimize according to <tt>formulator</tt>;
    * @param sink
    * @param form
    * @return success to optimize return true otherwise 
    * return false;
    */
   public abstract boolean optimize(
      OptimizerSink sink, Formulator formulator);
   
   /**
    * check if sink is compatible with this optimizer
    * @param sink
    * @return
    * if the sink is compatible with this optimizer --> true; 
    * otherwise --> false
    */
   public abstract boolean checkSink (OptimizerSink  sink);
   
   /**
    * check if formulator is compatible with this optimizer
    * @param F 
    * @return
    * if the <code>Formulator</code> is compatible with this optimizer --> true; 
    * otherwise --> false
    */
   public abstract boolean checkFormulator (Formulator F);
}
