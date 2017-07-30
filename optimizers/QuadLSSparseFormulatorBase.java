package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

/**
 * F = x^T * H * x + 2 x^T * q + c
 * <p>
 * H = extH + A^T * A
 * <p>
 * q = extq - A^T * b
 * 
 * 
 * @author KeyiTang
 *
 */
public abstract class QuadLSSparseFormulatorBase extends LOSparseFormulatorBase 
implements DenseQuadLSFormulator{

   MatrixNd extH = new MatrixNd ();
   VectorNd extq = new VectorNd ();
   double extc = 0;
   
   
   private MatrixNd tH = new MatrixNd ();
   private VectorNd tq = new VectorNd ();
   private double tc = 0;

   /**
    * H = extH + A^T * A
    * q = extq - A^T * b
    * 
    * F = x^T * H * x + 2 x^T * q + c
    */
   public boolean getQuadraticCostFunction (Matrix H, Vector q) {
      
      SparseBlockMatrix A = new SparseBlockMatrix ();
      VectorNd b = new VectorNd ();
      if (! getCostFunction (A, b)) {
         return false;
      }

      if (A.rowSize () == 0) {
         tH.setSize (mySize, mySize);
         tH.setZero ();
         tq.setSize (mySize);
         tq.setZero ();
      }
      else {
         tq.setSize (mySize);
         tH.setSize (mySize, mySize);
         SparseBlockMatrix.mulTransposeLeft (tH, A, A);
         A.mulTranspose (tq, b);
         tq.negate ();
         tc = b.dot (b) + extc;
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
   
   @Override
   public void addConstantTerm (double c) {
      extc += c;
   }
   
   public void addExternalTerm (MatrixNd H, VectorNd q, double c) {
      addExternalTerm (H, q);
      addConstantTerm (c);
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
      extc = 0;
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
   
   
   public double getExternalConstant () {
      return extc;
   }
   
   public double getConstantValue () {
      return tc;
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
   
   private boolean isZeros (VectorNd x) {
      for (int i = 0; i < x.size (); i++) {
         if (x.get (i) != 0) {
            return false;
         }
      }
      return true;
   }

}
