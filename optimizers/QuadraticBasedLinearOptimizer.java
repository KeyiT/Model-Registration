package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.core.inverse.LeastSquaresTermBase;
import artisynth.core.inverse.QPCostFunction;
import artisynth.core.inverse.QPTermBase;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector;
import maspack.matrix.VectorNd;

/**
 * 
 * @author KeyiTang
 *
 */
public class QuadraticBasedLinearOptimizer extends LinearOptimizer{

   QPCostFunction myQEFun = new QPCostFunction (); // quadratic energy function
   QPEnergyTerm myQETerm = new QPEnergyTerm (); // quadratic energy term
   
   private VectorNd oldX = new VectorNd ();
   private double oldE = Double.MAX_VALUE;
   private double newE = 0;
   private boolean enableEM = false;

   public QuadraticBasedLinearOptimizer () {
      myQEFun.addCostTerm (myQETerm);
   }
   
   @Override
   public boolean checkFormulator (Formulator form) {
      if (form instanceof QuadLSFormulator) {
         return true;
      }
      return false;
   }

   @Override
   protected VectorNd findSolution (LinearOptimizerFormulator form) {
      MatrixNd H = new MatrixNd ();
      VectorNd q = new VectorNd ();
      
      if (!checkFormulator (form)) {
         throw new ClassCastException ("Incompatible formulator"
         + "Formulator must be QuadLSFormulator");
      }
      
      QuadLSFormulator F = (QuadLSFormulator)form;
      
      if (! F.getQuadraticCostFunction (H, q)) {
         return null;
      }

      if (H.rowSize () != H.colSize ()) {
         throw new ImproperSizeException ("Hessian matrix is not square!");
      }
      if (q.size () != form.getDimension ()) {
         System.err.println (q.size ());
         System.err.println (form.getDimension ());
         throw new ImproperSizeException ("Incompatible size!");
      }
      if (H.rowSize () != q.size ()) {
         throw new ImproperSizeException ("Incompatible size!");
      }

      myQETerm.setSize (form.getDimension ());
      myQEFun.setSize (form.getDimension ());
      myQETerm.set (H, q);

      VectorNd output;
      try {
         output = myQEFun.solve (0, 1);
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println ("Unable to solve!");
         return null;
      }
      
      if (enableEM) {
         if (oldX.size () != form.getDimension ()) {
            throw new ImproperSizeException (
               "Incompatible initial value!");
         }
         
         oldE = myQETerm.computeEnergy (oldX);
         newE = myQETerm.computeEnergy (output);
      }

      return output;
   }
   
   public void setInitialValue (VectorNd value) {
         oldX.set (value);
   }
   
   public void getInitialValue (VectorNd value) {
      value.set (oldX);
   }
   
   public void measureEnergy (boolean enable) {
      enableEM = enable;
   }
   
   public boolean isEnergyMeasureEnabled () {
      return enableEM;
   }
   
   public double getInitalEnergy () {
      return oldE;
   }
   
   public double getOptimizedEnergy () {
      return newE;
   }


   public class QPEnergyTerm extends QPTermBase {

      @Override
      protected void compute (double t0, double t1) {
      }

      public void set(MatrixNd newQ, VectorNd newP) {
         Q.set (newQ);
         P.set (newP);
      }

      public void addQ(MatrixNd newQ) {
         if (Q.colSize () != newQ.colSize () || 
         Q.rowSize () != newQ.rowSize ()) {
            throw new ImproperSizeException("Incompatible Hessian matrix size");
         }
         Q.add (newQ);
      }

      public void addP(VectorNd newP) {
         if (P.size () != newP.size ()) {
            throw new ImproperSizeException("Incompatible propotional term size");
         }
         P.add (newP);
      }
      
      public double computeEnergy (VectorNd x) {
         if (x.size () != mySize) {
            throw new ImproperSizeException (
               "Incompatible vector size");
         }
         
         double energy = 0;
         // 2 x^T * q
         energy = x.dot (P);
         energy *= 2;
         // x^T H x
         VectorNd Hx = new VectorNd (mySize);
         Hx.mul (Q, x);
         energy += x.dot (Hx);
         
         return energy;
      }
   }

   // TODO: support linear constraints
   public class LinearEqualityConstraints extends LeastSquaresTermBase {
      @Override
      public int getRowSize () {
         return H.rowSize ();
      }

      @Override
      protected void compute (double t0, double t1) {
      }
   }

}
