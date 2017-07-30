package artisynth.models.swallowingRegistrationTool.correspondences;

import maspack.matrix.MatrixNd;

/**
 * Given two 3d point clouds, assemble them to multivariate statistical 
 * matrices; No correspondence map between two point clouds; 
 * @author KeyiTang
 *
 */
public interface Has3DDataPool {
   /**
    * Given <tt>n</tt> 3D source data, define a (<tt>n</tt> 
    * X <tt>3</tt>) data matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> X <tt>3</tt>) data matrix X for 
    * source data
    */
   public MatrixNd make3DSourceDataMatrix ();
   /**
    * Given <tt>n</tt> 3D target data, define a (<tt>n</tt> 
    * X <tt>3</tt>) data matrix X, which follows with multivariate 
    * statistical data form;
    * @return (<tt>n</tt> X <tt>3</tt>) data matrix X for 
    * target data
    */
   public MatrixNd make3DTargetDataMatrix ();
}
