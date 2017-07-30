package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.Vector;

public interface LinearOptimizerSink 
extends OptimizerSink{
   
   /**
    * 
    * @param output result comes out of optimizer
    */
   public void takeOutput (Vector output);
}
