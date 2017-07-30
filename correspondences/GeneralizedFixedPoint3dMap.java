package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.optimizers.GeneralizedPointInjectiveMapFormulator;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;

public abstract class GeneralizedFixedPoint3dMap<C extends VertexToPointCorrespondence> 
extends FixedPointPair3dMap<C> implements GeneralizedPointInjectiveMapFormulator{

   public GeneralizedFixedPoint3dMap () {
      super ();
   }

   /**
    * {@inheritDoc}
    * 
    * This method should be called after {@link #update}; 
    * <p>
    */
   @Override
   public SparseBlockMatrix createSourceGeneralizeMatrix () {
      
      int n = numInfos ();
      int num = myMap.size ();
      int idx = 0;
      
      int [] numbs = new int [num];
      for (int i = 0; i < num; i++) {
         numbs[i] = 3;
      }
      SparseBlockMatrix Cov = new SparseBlockMatrix (numbs, numbs);
      
      for(int i = 0; i < n; i++) {
         VertexToPointCorrespondence feature = 
         (VertexToPointCorrespondence)getInfo (i);
         
         if (feature.isMuted ()) {
            continue;
         }
         
         SparseBlockMatrix MatRow = createWeightMatrix (
            feature, true, idx);
         addBlockMatrixByRow (Cov, MatRow);
         
         idx ++;
      }
      
      return Cov;
   }

   /**
    * {@inheritDoc}
    * 
    * This method should be called after {@link #update}; 
    * <p>
    */
   @Override
   public SparseBlockMatrix createTargetGeneralizeMatrix () {
      int n = numInfos ();
      int num = myMap.size ();
      int idx = 0;
      
      int [] numbs = new int [num];
      for (int i = 0; i < num; i++) {
         numbs[i] = 3;
      }
      SparseBlockMatrix Cov = new SparseBlockMatrix ();
      
      for(int i = 0; i < n; i++) {
         VertexToPointCorrespondence feature = 
         (VertexToPointCorrespondence)getInfo (i);
         
         if (feature.isMuted ()) {
            continue;
         }
         
         SparseBlockMatrix MatRow = createWeightMatrix (
            feature, false, idx);
         addBlockMatrixByRow (Cov, MatRow);
         
         idx ++;
      }
      
      return Cov;
   }
   
   /**
    * create weight matrix for <tt>correspondence</tt>; 
    * 
    * @param correspondence
    * @param forSource if true returned weight matrix is used to assemble 
    * source-points-generalize matrix; otherwise returned weight matrix is 
    * used to assemble target-points-generalize matrix
    * @param idx 
    * @return 
    * <p>
    * Return3d matrix is a block matrix, which has one block in row, the number
    * of block in column is the same as the size of this injective map; Every block
    * is an instance of {@link Matrix3x3Block}; Returned matrix is used to assembling
    * the generalize matrix.
    */
   abstract protected SparseBlockMatrix createWeightMatrix (
      VertexToPointCorrespondence correspondence, 
      boolean forSource, int idx);
   
   
   
   private static void addBlockMatrixByRow (SparseBlockMatrix sbMat, SparseBlockMatrix matToAdd) {

      if (sbMat.numBlockRows () == 0) {
         sbMat.set (matToAdd);
         return;
      }
      int rn0 = matToAdd.numBlockRows ();
      int [] brs = new int [rn0];
      for (int i = 0; i < rn0; i++) {
         brs[i] = matToAdd.getBlockRowSize (i);
      }
      sbMat.addRows (brs, rn0);
      //System.out.println ("row: " + sbMat.numBlockRows ());
      
      int rib = sbMat.numBlockRows () - rn0;
      for (int i = 0; i < rn0; i++) {
         MatrixBlock blk = matToAdd.firstBlockInRow (i);
         while (blk != null) {
            sbMat.addBlock (rib+i, blk.getBlockCol (), blk.clone ());
            blk = blk.next ();
         }
      }
   }

}
