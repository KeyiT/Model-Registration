package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

public abstract class SparseQuadLSFormulatorBase extends LOSparseFormulatorBase 
implements SparseQuadLSFormulator{

   SparseBlockMatrix extH = new SparseBlockMatrix ();
   VectorNd extq = new VectorNd ();
   double extc = 0;

   private SparseBlockMatrix tH = new SparseBlockMatrix ();
   private VectorNd tq = new VectorNd ();
   private double tc = 0;

   /**
    * H = extH + A^T * A
    * q = extq - A^T * b
    * 
    * F = x^T * H * x + 2 x^T * q + c
    */

   @Override
   public boolean getQuadraticCostFunction (Matrix H, Vector q) {
      SparseBlockMatrix A = new SparseBlockMatrix ();
      VectorNd b = new VectorNd ();
      if (! getCostFunction (A, b)) {
         return false;
      }

      if (A.rowSize () == 0) {
         tH.set (new SparseBlockMatrix ());
         tq.setSize (mySize);
         tq.setZero ();
      }
      else {
         tq.setSize (mySize);
         tH.mulTransposeLeftToBlkMat (A, A);
         A.mulTranspose (tq, b);
         tq.negate ();
         tc = b.dot (b) + extc;
      }

      SparseBlockMatrix intH = new SparseBlockMatrix ();
      VectorNd intq = new VectorNd (0);
      if (! getInternalTerm (intH, intq)) {
         return false;
      }

      if (intq.size () != 0) {
         addHessian (tH, intH);
         addQ (tq, intq);
      }

      if (extH.rowSize () != 0) {
         addHessian (tH, extH);
      }

      if (extq.size () != 0) {
         addQ (tq, extq);
      }

      H.set (tH);
      q.set (tq);

      return true;
   }

   /**
    * make quadratic cost function for internal mapping goal; 
    * @param rH
    * @param rq
    * @return 
    */
   abstract protected boolean getInternalTerm (SparseBlockMatrix rH, VectorNd rq);


   @Override
   public void addQuadraticTerm (SparseBlockMatrix H) {
      checkSize (H);
      addHessian (extH, H);
   }

   @Override
   public void addPropotionalTerm (VectorNd q) {
      checkSize (q);
      addQ (extq, q);
   }

   @Override
   public void addConstantTerm (double c) {
      extc += c;
   }

   private void clearQuadraticTerm () {
      extH.set (new SparseBlockMatrix ());
   }

   private void clearProportionalTerm () {
      extq.setZero ();
      extq.setSize (mySize);
   }

   public void clearExternalTerm () {
      clearQuadraticTerm();
      clearProportionalTerm ();
      extc = 0;
   }

   public void getExternalTerm (SparseBlockMatrix H, VectorNd q) {
      if (H != null) {
         H.set (extH);
      }
      if (q != null) {
         q.set (extq);
      }
   }

   /**
    * F = x^T * H * x + 2 x^T * q + c
    * @param x
    * @return
    */
   public double computeEnergy (VectorNd x) {
      double e = 0;
      checkSize (x);
      // proportional term
      // and constant term
      if (isZeros (x)) {
         return tc;
      }
      e = 2 * x.dot (tq) + tc;
      // quadratic term
      VectorNd Hx = new VectorNd ();
      tH.mul (Hx, x);
      e += Hx.dot (x);

      return e;
   }



   public double computeDecreasingRate (
      VectorNd x0, VectorNd x1) {
      double e0 = computeEnergy (x0);
      double e1 = computeEnergy (x1);
      double ratio = e0 - e1;
      ratio = ratio / e0;
      return ratio;
   }


   /*
    * assume matrix size is right
    */
   private void addHessian (SparseBlockMatrix H, SparseBlockMatrix HToAdd) {
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


   private void checkSize (SparseBlockMatrix H, VectorNd q) {
      checkSize (H);
      checkSize (q);
   }

   private void checkSize (SparseBlockMatrix H) {
      if (H.rowSize () != mySize || H.colSize () != mySize) {
         throw new ImproperSizeException ("Incompatible Hessian matrix");
      }
   }

   private void checkSize (VectorNd q) {
      if (q.size () != mySize) {
         throw new ImproperSizeException ("Incompatible vector");
      }
   }

   private boolean isZeros (VectorNd x) {
      for (int i = 0; i < x.size (); i++) {
         if (x.get (i) != 0) {
            return false;
         }
      }
      return true;
   }

}
