package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.MatrixNd;

public interface PointInjectiveMapFormulator extends PointMapFormulator{
   /**
    * Given <tt>n</tt> p-D source data, define a (<tt>n</tt> 
    * x <tt>p</tt>) matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> x <tt>p</tt>) data matrix X for 
    * source data
    * 
    * <p>
    * Returned matrix will has the same row size with the matrix returned by 
    * {@link #makeTargetDataMatrix()} and {@link #makeTargetedChangeMatrix()}
    */
   @Override
   public MatrixNd makeSourceDataMatrix ();
   
   /**
    * Given <tt>n</tt> p-D target data, define a (<tt>n</tt> 
    * x <tt>p</tt>) matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> x <tt>p</tt>) data matrix X for 
    * target data
    * 
    * <p>
    * Returned matrix will has the same row size with the matrix returned by 
    * {@link #makeSourceDataMatrix} and {@link #makeTargetedChangeMatrix}
    */
   @Override
   public MatrixNd makeTargetDataMatrix ();
   
   /**
    * Given <tt>n</tt> p-D targeted change data, define a (<tt>n</tt> 
    * x <tt>p</tt>) matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> x <tt>p</tt>) data matrix X for 
    * targeted change
    * 
    * <p>
    * Returned matrix will has the same row size with the matrix returned by 
    * {@link #makeTargetDataMatrix} and {@link #makeSourceMatrix}
    */
   public MatrixNd makeTargetedChangeMatrix ();
   
}
