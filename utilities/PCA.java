package artisynth.models.swallowingRegistrationTool.utilities;

import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class PCA {

   public PCA () {
      // TODO Auto-generated constructor stub
   }

   /**
    * compute data sample mean
    * @param meanVec if null, return sample mean vector; 
    * size = dimension;
    * @param Data sample matrix, N X D; 
    * rowSize = sample size; 
    * colSize = dimension;
    */
   public static void computeDataMean (VectorNd meanVec, MatrixNd Data) {
      VectorNd mean = new VectorNd();
      mean.setSize (Data.colSize ());
      for (int i = 0; i < Data.colSize (); i++) {
         for (int j = 0; j < Data.rowSize (); j++) {
            mean.set (i, mean.get (i) + Data.get (j, i));
         }
      }
      mean.scale (1.0/Data.rowSize ());
      meanVec.set (mean);
   }


   /**
    * translation data to zero mean
    * @param cenData if null return zero-mean data; 
    * rowSize = sample size; 
    * colSize = dimension;
    * @param Data sample matrix, N X D; 
    * rowSize = sample size; 
    * colSize = dimension;
    */
   public static void centralizeData(MatrixNd CenData, MatrixNd Data) {
      VectorNd mean = new VectorNd ();
      computeDataMean(mean, Data);
      CenData.setSize (Data.rowSize (), Data.colSize ());
      for (int i = 0; i < Data.rowSize (); i++) {
         for (int j = 0; j < Data.colSize (); j++) {
            CenData.set (i, j, Data.get (i, j)-mean.get (j));
         }
      }
   }

   /**
    * translation data to zero mean
    * @param cenData if null return zero-mean data; 
    * rowSize = sample size; 
    * colSize = dimension;
    * @param Data sample matrix, N X D; 
    * rowSize = sample size; 
    * colSize = dimension;
    */
   public static void centralizeData(MatrixNd CenData, VectorNd meanVec, MatrixNd Data) {
      computeDataMean(meanVec, Data);
      CenData.setSize (Data.rowSize (), Data.colSize ());
      for (int i = 0; i < Data.rowSize (); i++) {
         for (int j = 0; j < Data.colSize (); j++) {
            CenData.set (i, j, Data.get (i, j)-meanVec.get (j));
         }
      }
   }

   /**
    * compute covariance of <tt>Data</tt>
    * @param Cov covariance matrix;
    * N X N;
    * @param Data sample matrix, N X D; 
    * rowSize = sample size; 
    * colSize = dimension;
    */
   public static void computeCov (MatrixNd Cov, MatrixNd Data) {
      MatrixNd Cen = new MatrixNd();
      centralizeData(Cen, Data);
      Cov.set (Cen);
      Cov.mulTransposeLeft (Cov, Cen);
      Cov.scale (1.0/Data.rowSize ());
   }

   /**
    * compute covariance of <tt>Data</tt>
    * @param Cov covariance matrix;
    * N X N;
    * @param Data sample matrix, N X D; 
    * rowSize = sample size; 
    * colSize = dimension;
    */
   public static void computeCov (MatrixNd Cov, MatrixNd CenData, VectorNd meanVec, MatrixNd Data) {
      centralizeData(CenData, meanVec, Data);
      Cov.set (CenData);
      Cov.mulTransposeLeft (Cov, CenData);
      Cov.scale (1.0/Data.rowSize ());
   }


   /**
    * 
    * @param PCs
    * @param CSize
    * @param Data
    */
   public static void computePDs(MatrixNd PDs, VectorNd CSize, MatrixNd Data) {
      SVDecomposition SVD = new SVDecomposition();
      MatrixNd Cov = new MatrixNd ();
      computeCov(Cov, Data);

      SVD.factor (Cov);
      // eigenvectors - principle directions
      if (PDs != null) {
         PDs.set (SVD.getU ());
      }
      // eigenvalues - principle direction sizes
      if (CSize != null) {
         CSize.set (SVD.getS ());
      }
   }

   /**
    * 
    * @param PCs
    * @param CSize
    * @param Data
    */
   public static void computePDs(MatrixNd PDs, VectorNd CSize, MatrixNd Cov, MatrixNd CenData, 
      VectorNd meanVec, MatrixNd Data) {
      
      SVDecomposition SVD = new SVDecomposition();
      
      MatrixNd tmpCov = new MatrixNd ();
      MatrixNd tmpCenData = new MatrixNd ();
      VectorNd mean = new VectorNd ();
      computeCov(tmpCov, tmpCenData, mean, Data);

      SVD.factor (tmpCov);
      // eigenvectors - principle directions
      if (PDs != null) {
         PDs.set (SVD.getU ());
      }
      // eigenvalues - principle direction sizes
      if (CSize != null) {
         CSize.set (SVD.getS ());
      }
      if (Cov != null) {
         Cov.set (tmpCov);
      }
      if (CenData != null) {
         CenData.set (tmpCenData);
      }
      if (meanVec != null) {
         meanVec.set (mean);
      }
   }


   /**
    * get bounding box of mesh according to it's PCA;
    * @param Data
    * @param resBBV bounding box 8 vertexes
    * @param singularValue
    * @param marginRatio A negative value will trigger 
    * default value 0.025 being applied;
    */
   public static RigidTransform3d computeBBVs (
      MatrixNd Data, Point3d [] resBBV, 
      Vector3d singularValue, double marginRatio) {

      MatrixNd U = new MatrixNd();
      VectorNd mean = new VectorNd ();
      if (singularValue == null) {
         computePDs (U, null, null, null, mean, Data);
      }
      else {
         VectorNd sig = new VectorNd ();
         computePDs (U, sig, null, null, mean, Data);
         singularValue.set (sig);
      }
      
      
      RigidTransform3d temX1W = new RigidTransform3d();
      temX1W.R.set (U);
      Point3d centroid = new Point3d();
      centroid.set (mean);

      /*
      if (temX1W.R.determinant() < 0) {
         // flip x axis to correct
         temX1W.R.m00 = -temX1W.R.m00;
         temX1W.R.m10 = -temX1W.R.m10;
         temX1W.R.m20 = -temX1W.R.m20;
      }*/

      // get Mesh Bounding Box
      Point3d pnt = new Point3d();
      double huge = Double.MAX_VALUE;
      Vector3d temMax = new Vector3d (-huge, -huge, -huge);
      Vector3d temMin = new Vector3d ( huge,  huge,  huge);
      for (int i=0; i< Data.rowSize (); i++) {
         Point3d data = new Point3d ();
         Data.getRow (i, data);
         pnt.inverseTransform (temX1W.R, data);
         pnt.updateBounds (temMin, temMax);
      }


      Vector3d diaVec = new Vector3d();
      diaVec.sub (temMax, temMin);
      if (marginRatio < 0) {
         diaVec.scale (0.025);
      }
      else {
         diaVec.scale (marginRatio);
      }
      
      temMax.add (diaVec);
      temMin.sub (diaVec);

      // get centroid of the mesh
      centroid.add (temMax, temMin);
      centroid.scale (0.5);
      temX1W.p.set (centroid);

      // get Mesh Bounding Box 8 Vertexes
      // x is the highest order position
      // z is the lowest order position
      Vector3d [] temBBV = new Vector3d [2];
      temBBV[0] = temMin;
      temBBV[1] = temMax;
      int g = 0;
      Vector3d temVec = new Vector3d();
      for (int i = 0; i <=1 ; i++) {
         for (int j = 0; j <= 1; j++) {
            for (int k = 0; k <= 1; k++) {
               temVec.set (temBBV[i].x, temBBV[j].y, temBBV[k].z);
               temVec.transform (temX1W.R);
               resBBV[g] = new Point3d ();
               resBBV[g].set (temVec);
               g++;
            }
         }
      }
      temMin.transform (temX1W.R);
      temMax.transform (temX1W.R);

      temX1W.p.set (temBBV[0]);

      return temX1W;
   }
   
   /**
    * get bounding box of mesh according to it's PCA; 
    * Default margin ratio value 0.025 is applied; 
    * @param Data
    * @param resBBV
    * @param singularValue
    * @return
    */
   public static RigidTransform3d computeBBVs (
      MatrixNd Data, Point3d [] resBBV, 
      Vector3d singularValue) {
      return computeBBVs (Data, resBBV, singularValue, -1);
   }
   
   
   /**
    * get bounding box of mesh according to it's PCA; 
    * Default margin ratio value 0.025 is applied; 
    * @param argMesh input mesh
    * @param resBBV bounding box 8 vertexes
    */
   public static RigidTransform3d computeBBVs (MeshBase argMesh, Point3d [] resBBV, Vector3d singularValue) {

      MatrixNd Data = new MatrixNd (argMesh.getVertices ().size (), 3);
      int idx = 0;
      for (Vertex3d vtx : argMesh.getVertices ()) {
         Point3d pnt = new Point3d (vtx.getWorldPoint ());
         Data.setRow (idx++, pnt);
      }

      return computeBBVs (Data, resBBV, singularValue);
   }
   
   
   /**
    * get bounding box of mesh according to it's PCA; 
    * Default margin ratio value 0.025 is applied; 
    * @param argMesh input mesh
    * @param resBBV bounding box 8 vertexes
    */
   public static RigidTransform3d computeBBVs (MeshBase argMesh, Point3d endBBV, Vector3d singularValue) {
      Point3d [] resBBV = new Point3d [8];
      RigidTransform3d Xmw = computeBBVs (argMesh, resBBV, singularValue);
      if (endBBV != null) {
         endBBV.set (resBBV[7]);
      }
      return Xmw;
   }


}
