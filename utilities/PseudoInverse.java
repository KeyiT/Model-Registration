package artisynth.models.swallowingRegistrationTool.utilities;

import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.SVDecomposition;
import maspack.matrix.VectorNd;

public class PseudoInverse {

   private boolean svdMade = false;
   private boolean inverted = false;

   private int myColSize = 0;
   private int myRowSize = 0;

   MatrixNd myInv = new MatrixNd ();
   SVDecomposition mysvd = new SVDecomposition();

   public PseudoInverse () {
   }


   /**
    * Creates a matrix whose size and elements are the same as an existing
    * Matrix.
    * 
    * @param M
    * matrix object to be copied.
    */
   public PseudoInverse (Matrix M) {
      myColSize = M.colSize ();
      myRowSize = M.rowSize ();
      checkForBuffer();
      refactor(M);
      svdMade = true;
   }


   /**
    * Creates a matrix which is a copy of an existing one.
    * 
    * @param M
    * matrix to be copied.
    */
   public PseudoInverse (MatrixNd M) {
      myColSize = M.colSize ();
      myRowSize = M.rowSize ();
      checkForBuffer();
      refactor(M);
      svdMade = true;
   }

   /**
    * Creates a matrix from a two dimensional array of doubles. The matrix size
    * will be determined by the size of this array.
    * 
    * @param values
    * element values for the new matrix
    */
   public PseudoInverse (double[][] values) {
      MatrixNd M = new MatrixNd(values);
      myColSize = M.colSize ();
      myRowSize = M.rowSize ();
      checkForBuffer();
      refactor(M);
      svdMade = true;
   }

   public void set(Matrix M) {
      myColSize = M.colSize ();
      myRowSize = M.rowSize ();
      checkForBuffer();
      refactor(M);
      svdMade = true;
   }


   private void checkForBuffer() {
      if (myRowSize == 0 || myColSize == 0) {
         throw new ImproperStateException(
         "Invalid Matrix");
      }
   }

   private void checkForSVD () {
      if (svdMade = false) {
         throw new ImproperStateException(
         "SVD not initialized");
      }
      if(mysvd.getS () == null ||
      mysvd.getU () == null ||
      mysvd.getV () == null) {
         throw new ImproperStateException(
         "SVD was not initialized properly");
      }
   }

   private void checkForInv () {
      if (inverted == false) {
         throw new ImproperStateException(
         "Pseudoinverse was not performed");
      }
   }

   private void refactor(Matrix M) {
      mysvd.factor (M);
   }

   /**
    * pseudo-inverse
    */
   public void pseInvert() {
      checkForSVD();
      // singular value
      VectorNd sig = mysvd.getS ();
      MatrixNd U = mysvd.getU ();
      MatrixNd V = mysvd.getV ();

      int N = 0;
      if (myRowSize >= myColSize) {
         N = myColSize;  
      } else {
         N = myRowSize;
      }

      // size of pseudoinverse matrix should be 
      // same as the transpose of original matrix
      myInv.setSize (myColSize, myRowSize);

      // determine the rank
      int idx = 0;
      while (sig.get (N-1-idx) == 0) {
         idx++;
      }
      N = N-idx;

      // V * S^+ * U^T
      double val = 0;

      for (int i = 0; i < myColSize; i++) {
         for (int j = 0; j < myRowSize; j++) {
            for (int n = 0; n < N; n++) {
               val += V.get (i, n) * U.get (j, n) / sig.get (n);
            }
            myInv.set (i, j, val);
            val = 0;
         }
      }

      inverted = true;
   }
   
   public MatrixNd getInv () {
      return myInv;
   }
}
