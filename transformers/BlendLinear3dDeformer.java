package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.List;

import artisynth.models.swallowingRegistrationTool.optimizers.LinearOptimizerSink;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public abstract class BlendLinear3dDeformer extends 
RegistrationDeformer implements LinearOptimizerSink
{

   abstract public Matrix makeBasis (Point3d pnt);

   abstract public Matrix makeBasis (MatrixNd pnts);

   abstract public Matrix3d getF (Vector3d r);

   abstract public void setCoef (VectorNd P);

   abstract public void getCoef (VectorNd P);

   public void setCoef (MatrixNd P) {
      if (P.colSize () != 3) {
         throw new IllegalArgumentException (
            "Column size must be 3");
      }

      VectorNd myP = new VectorNd ();

      for (int i = 0; i < P.rowSize (); i++) {
         for (int j = 0; j < P.colSize (); j++) {
            myP.set (i*3 + j, P.get (i, j));
         }
      }

      setCoef (myP);
   }

   public void getCoef (MatrixNd P) {
      VectorNd myP = new VectorNd ();
      getCoef(myP);
      P.set (convertMapVector2Matrix (myP));
   }

   private Vector3d evalDisplacement (Point3d r) {

      Matrix bas = makeBasis(r);
      VectorNd vr = new VectorNd ();
      VectorNd myP = new VectorNd ();
      getCoef (myP);
      bas.mul (vr, myP);

      return (new Vector3d(vr));
   }

   /**
    * {@inheritDoc}
    */
   public void getDeformation (Vector3d p, Matrix3d F, Vector3d r) {
      if (p != null) {
         p.set (evalDisplacement(new Point3d (r)));
      }
      if (F != null) {
         F.set (getF(r));
      }
   }

   /**
    * {@inheritDoc}
    * 
    * <p>
    * <tt>output</tt> value is taken as the change of coefficients; 
    * The sum of output value and old coefficients value will be assign
    * to the coefficient vector.
    */
   @Override
   public void takeOutput (Vector output) {
      if (output == null) {
         return;
      }
      
      VectorNd myP = new VectorNd ();
      getCoef (myP);
      
      if (myP.size () != output.size ()) {
         throw new ImproperSizeException (
         "Incompatible optimization output!");
      }

      VectorNd solution;
      if ( output instanceof VectorNd) {
         solution = (VectorNd) output;
      }
      else {
         solution = new VectorNd ();
         solution.set (output);
      }

      myP.add (solution);
      setCoef (myP);
   }

   private MatrixNd convertMapVector2Matrix (VectorNd P) {
      if (P.size () % 3 != 0) {
         throw new ImproperSizeException ("Incompatible dimension");
      }
      int idx = 0;
      MatrixNd Pr = new MatrixNd (P.size ()/3, 3);
      for (int i = 0; i < Pr.rowSize (); i++) {
         for (int j = 0; j < 3; j++) {
            Pr.set (i, j, P.get (idx++));
         }
      }

      return Pr;
   }

}

