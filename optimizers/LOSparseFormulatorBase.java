package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixBlock;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

/**
 * Linear optimizer sparse formulator 
 * 
 * @author KeyiTang
 *
 */
public abstract class LOSparseFormulatorBase implements LinearOptimizerFormulator{
   
   private SparseBlockMatrix extA = new SparseBlockMatrix ();
   private VectorNd extb = new VectorNd ();

   protected int mySize = 0;

   @Override
   public boolean getCostFunction (Matrix A, Vector b) {

      SparseBlockMatrix intA = new SparseBlockMatrix ();
      VectorNd intb = new VectorNd ();
      
      if (!updateInternalGoals (intA, intb)) {
         return false; 
      }
      
      if (intA.rowSize () == 0 || intb.size () == 0) {
         if (extA.rowSize () != 0 ) {
            A.set (extA);
            b.set (extb);
         }
         else {
            A.set (new SparseBlockMatrix ());
            b.setSize (0);
         }
         return true;
      }
      
      checkSize (intA, intb);
      
      if (extA.rowSize () != 0) {
         addBlockMatrixByRow (intA, extA);
         int goalNum = intb.size ();
         intb.setSize (goalNum+extb.size ());
         intb.setSubVector (goalNum, extb);
      }
      
      A.set (intA);
      b.set (intb);
      
      return true;
   }
   
   @Override
   public void setDimension (int d) {
      mySize = d;
   }
   
   @Override
   public int getDimension () {
      return mySize;
   }

   
   public void addExternalGoals (
      SparseBlockMatrix A, Vector b) {

      checkSize (A, b);
      
      addBlockMatrixByRow (extA, A);

      int goalNum = extb.size ();
      extb.setSize (goalNum+b.size ());
      extb.setSubVector (goalNum, b);
   }
   
   /**
    * This method assume corresponding vector b is a zero vector;
    * @param A
    */
   public void addExternalGoals (
      SparseBlockMatrix A) {
      
      checkSize (A);
      addBlockMatrixByRow (extA, A);
      
      int goalNum = extb.size ();
      int num = A.rowSize ();
      extb.setSize (goalNum+num);
      extb.setSubVector (goalNum, new VectorNd (num));
   }
   
   public void clearExternalGoal () {
      extA.set (new SparseBlockMatrix ());
      extb.setZero ();
      extb.setSize (0);
   }
   
   public void getExternalGoals (SparseBlockMatrix A, VectorNd b) {
      A.set (extA);
      b.set (extb);
   }
   
   /**
    * update internal linear mapping goals according to <code>CloudMap</code>;
    * 
    * @param A
    * @param b
    * @return
    */
   public abstract boolean updateInternalGoals (SparseBlockMatrix A, VectorNd b);
   
   
   
   private void checkSize (Matrix A) {
      if (A.colSize () != mySize) {
         throw new ImproperSizeException ("Incompatible matrix size");
      }
   }
   
   private void checkSize (Matrix A, Vector b) {
      checkSize (A);
      if (b.size () != A.rowSize ()) {
         throw new ImproperSizeException ("Incompatible size");
      }
   }
   
   protected void addBlockMatrixByRow (SparseBlockMatrix sbMat, SparseBlockMatrix matToAdd) {

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
