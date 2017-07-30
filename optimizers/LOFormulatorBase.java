package artisynth.models.swallowingRegistrationTool.optimizers;


import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

public abstract class LOFormulatorBase implements LinearOptimizerFormulator{
   
   private MatrixNd extA = new MatrixNd ();
   private VectorNd extb = new VectorNd ();

   protected int mySize = 0;

   @Override
   public boolean getCostFunction (Matrix A, Vector b) {

      MatrixNd intA = new MatrixNd ();
      VectorNd intb = new VectorNd ();
      
      if (!updateInternalGoals (intA, intb)) {
         return false; 
      }
      
      checkSize (intA, intb);
      
      if (extA.rowSize () != 0) {
         int goalNum = intb.size ();
         addMatrixByRow (intA, extA);
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
      MatrixNd A, Vector b) {

      checkSize (A, b);
      
      addMatrixByRow (extA, A);

      int goalNum = extb.size ();
      extb.setSize (goalNum+b.size ());
      extb.setSubVector (goalNum, b);
   }
   
   /**
    * This method assume corresponding vector b is a zero vector;
    * @param A
    */
   public void addExternalGoals (
      MatrixNd A) {
      
      checkSize (A);
      addMatrixByRow (extA, A);
      
      int goalNum = extb.size ();
      int num = A.rowSize ();
      extb.setSize (goalNum+num);
      extb.setSubVector (goalNum, new VectorNd (num));
   }
   
   public void clearExternalGoal () {
      extA.setZero ();
      extA.setSize (0, 0);
      extb.setZero ();
      extb.setSize (0);
   }
   
   /**
    * update internal linear mapping goals according to <code>CloudMap</code>;
    * 
    * @param A
    * @param b
    * @return
    */
   public abstract boolean updateInternalGoals (MatrixNd A, VectorNd b);
   
   
   /*
    * assume size is compatible
    */
   private static void addMatrixByRow (MatrixNd hostM, MatrixNd addM) {
      int goalNum = hostM.rowSize ();
      hostM.setSize (goalNum+addM.rowSize (), hostM.colSize ());
      hostM.setSubMatrix (goalNum, 0, hostM);
   }
   
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

}
