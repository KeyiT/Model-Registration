package artisynth.models.swallowingRegistrationTool.transformers;

import artisynth.models.swallowingRegistrationTool.utilities.*;
import maspack.geometry.DeformationTransformer;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public abstract class LinearModelDeformer extends DeformationTransformer 
implements HasRenderableModel, Updatable{

   public LinearModelDeformer () {
   }

   public abstract VectorNd computeKernels (Vector3d X);
   /**
    * Kernel size X 3
    */
   public abstract MatrixNd getMappingMatrix ();

   /**
    * Kernel size X 3
    */
   public abstract void setMappingMatrix (MatrixNd P);

   /**
    * Deformation Matrix F
    */
   public abstract Matrix3d getF(Vector3d r);


   /**
    * column size of Xs must be 3; 
    * 
    * column size of the return matrix is kernel size; 
    * row size of the the return matrix is same as the input matrix
    */
   public MatrixNd computeKernels (MatrixNd Xs) {
      if (Xs.colSize () != 3) {
         throw new ImproperSizeException("Incompatible Size");
      }

      Vector3d tmpX = new Vector3d();

      tmpX.x = Xs.get (0, 0);
      tmpX.y = Xs.get (0, 1);
      tmpX.z = Xs.get (0, 2);
      VectorNd ker = computeKernels(tmpX);
      MatrixNd kers = new MatrixNd (Xs.rowSize (), ker.size ());
      kers.setRow (0, ker);
      for (int i = 1; i < Xs.rowSize (); i++) {
         tmpX.x = Xs.get (i, 0);
         tmpX.y = Xs.get (i, 1);
         tmpX.z = Xs.get (i, 2);
         ker = computeKernels(tmpX);
         kers.setRow (i, ker);
      }
      return kers;
   }


   private Vector3d computeDisplacement (Vector3d r) {
      VectorNd ker = computeKernels(r);
      MatrixNd map = new MatrixNd (getMappingMatrix());

      if (map.rowSize () != ker.size () || 
      map.colSize () != 3) {
         throw new ImproperSizeException("Incompatible kernels and mapping matrix");
      }

      map.transpose ();
      VectorNd tmpV = new VectorNd();
      tmpV.mul (map, ker);
      return (new Vector3d(tmpV));
   }

   /**
    * {@inheritDoc}
    */
   public void getDeformation (Vector3d p, Matrix3d F, Vector3d r) {
      if (p != null) {
         p.set (computeDisplacement(new Vector3d(r)));
      }
      if (F != null) {
         F.set (getF(r));
      }

   }




}
