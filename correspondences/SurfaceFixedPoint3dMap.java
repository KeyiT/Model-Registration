package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.geometry.Face;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.Vector3d;

public class SurfaceFixedPoint3dMap<C extends VertexToPointCorrespondence>
extends GeneralizedFixedPoint3dMap<C> {

   double srcCov = 0.1;

   public SurfaceFixedPoint3dMap () {
      super ();
   }

   /**
    * {@inheritDoc}
    * 
    * @return
    * 
    * {@inheritDoc}
    * 
    * <p>
    * This method creates covariance matrix for each source point
    * based on their surface normal and assemble them into a 
    * diagonal block matrix;
    */
   @Override
   protected SparseBlockMatrix createWeightMatrix (
      VertexToPointCorrespondence correspondence, boolean forSource, int index) {

      int num = myMap.size ();

      int [] numbs = new int [num];
      for (int i = 0; i < num; i++) {
         numbs[i] = 3;
      }
      int [] b = {3};

      SparseBlockMatrix Cov = new SparseBlockMatrix (b, numbs);

      Matrix3x3Block cov = new Matrix3x3Block ();
      Vector3d nrm = new Vector3d ();

      if ((correspondence.getDirection () && forSource) 
      || (!correspondence.getDirection () && !forSource)) {
         Vertex3d vtx = correspondence.getVertex ();
         if (! vtx.computeAngleWeightedNormal (nrm)) {
            cov.setIdentity ();
            setBlock(Cov, cov, index);
            return Cov;
         }
      }
      else {
         if (correspondence.getFeature () instanceof Vertex3d) {
            Vertex3d vtx = (Vertex3d)correspondence.getFeature ();
            if (! vtx.computeAngleWeightedNormal (nrm)) {
               cov.setIdentity ();
               setBlock(Cov, cov, index);
               return Cov;
            }
         }
         else if (correspondence.getFeature () instanceof Face) {
            Face face = (Face)correspondence.getFeature ();
            face.computeNormal (nrm);
         }
         else {
            cov.setIdentity ();
            setBlock(Cov, cov, index);
            return Cov;
         }
      }


      Matrix3x3Block R = new Matrix3x3Block ();
      nrm.normalize ();
      if (nrm.containsNaN () || nrm.norm () == 0) {
         cov.setIdentity ();
         setBlock(Cov, cov, index);
         return Cov;
      }

      Vector3d vec0 = new Vector3d ();
      int idxMax = nrm.maxAbsIndex ();
      int idxMin = nrm.minAbsIndex ();
      int idxMid = 1;

      if (idxMax == idxMin) {
         idxMin = 0;
         idxMid = 1;
         idxMax = 2;
      }
      else {
         idxMid = 3 - idxMax -idxMin;
      }

      vec0.set (idxMax, 1.0 / nrm.get (idxMax));

      if (Math.abs (nrm.get (idxMid)) > 1E-16) {
         vec0.set (idxMid, - 1.0/nrm.get (idxMid));
      }
      vec0.normalize ();

      Vector3d vec1 = new Vector3d ();
      vec1.cross (nrm, vec0);

      R.setColumn (0, nrm);
      R.setColumn (1, vec0);
      R.setColumn (2, vec1);

      vec0.scale (srcCov);
      vec1.scale (srcCov);
      cov.setColumn (0, nrm);
      cov.setColumn (1, vec0);
      cov.setColumn (2, vec1);
      cov.mulTranspose (R);
      setBlock (Cov, cov, index);

      return Cov;
   }

   private void setBlock (SparseBlockMatrix Mat, Matrix3x3Block blk, int idx) {
      Mat.addBlock (0, idx, blk);
   }

}
