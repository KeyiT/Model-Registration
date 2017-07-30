package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.MatrixNd;

public interface PointMapFormulator extends Formulator{
   /**
    * Given <tt>n</tt> p-D source data, define a (<tt>n</tt> 
    * x <tt>p</tt>) matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> x <tt>p</tt>) data matrix X for 
    * source data
    */
   public MatrixNd makeSourceDataMatrix ();
   
   /**
    * Given <tt>n</tt> p-D target data, define a (<tt>n</tt> 
    * x <tt>p</tt>) matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> x <tt>p</tt>) data matrix X for 
    * target data
    */
   public MatrixNd makeTargetDataMatrix ();
}
