package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

/**
 * F = x^T * H * x + 2 x^T * q
 * <p>
 * H = extH + A^T * A
 * <p>
 * q = extq - A^T * b
 * 
 * 
 * @author KeyiTang
 *
 */
public abstract class QuadLSFormulatorBase extends LOFormulatorBase
implements DenseQuadLSFormulator{

   MatrixNd extH = new MatrixNd ();
   VectorNd extq = new VectorNd ();

   /**
    * H = extH + A^T * A
    * q = extq - A^T * b
    * 
    * f = x^T * H * x + 2 x^T * q
    */
   public boolean getQuadraticCostFunction (Matrix H, Vector q) {
      MatrixNd A = new MatrixNd ();
      VectorNd b = new VectorNd ();
      if (! getCostFunction (A, b)) {
         return false;
      }

      MatrixNd intH  = new MatrixNd ();
      VectorNd intq = new VectorNd ();

      intH.mulTransposeLeft (A, A);
      A.mulTranspose (intq, b);
      intq.negate ();


      if (extH.rowSize () != 0) {
         addHessian (intH, extH);
      }

      if (extq.size () != 0) {
         addQ (intq, extq);
      }

      H.set (intH);
      q.set (intq);

      return true;
   }

   @Override
   public void addQuadraticTerm (MatrixNd H) {
      checkSize (H);
      addHessian (extH, H);
   }

   @Override
   public void addPropotionalTerm (VectorNd q) {
      checkSize (q);
      addQ (extq, q);
   }

   public void addExternalTerm (MatrixNd H, VectorNd q) {
      checkSize (H, q);
      addHessian (extH, H);
      addQ (extq, q);
   }

   private void clearQuadraticTerm () {
      extH.setZero ();
      extH.setSize (0, 0);
   }

   private void clearProportionalTerm () {
      extq.setZero ();
      extq.setSize (0);
   }

   public void clearExternalTerm () {
      clearQuadraticTerm();
      clearProportionalTerm ();
   }

   /*
    * assume matrix size is right
    */
   private void addHessian (MatrixNd H, MatrixNd HToAdd) {
      if (H.rowSize () == 0) {
         H.set (HToAdd);
         return;
      }

      H.add (HToAdd);
   }
   /*
    * assume vector size is right
    */
   private void addQ (VectorNd q, VectorNd qToAdd) {
      if (q.size () == 0) {
         q.set (qToAdd);
      }

      q.add (qToAdd);
   }


   private void checkSize (MatrixNd H, VectorNd q) {
      checkSize (H);
      checkSize (q);
   }

   private void checkSize (MatrixNd H) {
      if (H.rowSize () != mySize || H.colSize () != mySize) {
         throw new ImproperSizeException ("Incompatible Hessian matrix");
      }
   }

   private void checkSize (VectorNd q) {
      if (q.size () != mySize) {
         throw new ImproperSizeException ("Incompatible vector");
      }
   }

   public void getExternalTerm (MatrixNd H, VectorNd q) {
      if (H != null) {
         H.set (extH);
      }
      if (q != null) {
         q.set (extq);
      }
   }

   protected MatrixNd getExternalHessian () {
      return extH;
   }

   protected VectorNd getExternalProp () {
      return extq;
   }

}
