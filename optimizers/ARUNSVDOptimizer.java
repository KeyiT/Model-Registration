package artisynth.models.swallowingRegistrationTool.optimizers;

import artisynth.models.swallowingRegistrationTool.utilities.PCA;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SVDecomposition;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/**
 * Reference : Least-Squares Fitting of Two 3-D Point Sets, 
 * K. S  ARUN et. al. 1987; 
 * 
 * @author KeyiTang
 *
 */
public class ARUNSVDOptimizer extends Optimizer{
   
   private boolean scalingEnabled = true;
   
   public void enableScaling (boolean enable) {
      scalingEnabled = enable;
   }
   
   public boolean isScalingEnabled () {
      return scalingEnabled;
   }

   @Override
   public boolean optimize (OptimizerSink sink, Formulator formulator) {
      if (!checkSink(sink)) {
         throw new IllegalArgumentException ("Incompatible sink!");
      }
      
      if (! checkFormulator (formulator)) {
         throw new IllegalArgumentException ("Incompatible formulator!");
      }
      
      PointInjectiveMapFormulator form = (PointInjectiveMapFormulator) formulator;
      AffineSink asink = (AffineSink) sink;
      
      asink.takeOutput (
         fitRigid(form.makeSourceDataMatrix (), 
                  form.makeTargetDataMatrix (), 
                  scalingEnabled));
      return true;
   }

   @Override
   public boolean checkSink (OptimizerSink sink) {
      if (sink instanceof AffineSink) {
         return true;
      }
      return false;
   }

   @Override
   public boolean checkFormulator (Formulator F) {
      if (F instanceof PointInjectiveMapFormulator) {
         return true;
      }
      return false;
   }
   
   
   public static AffineTransform3d fitRigid (MatrixNd srcPnts, 
      MatrixNd tgtPnts, boolean scale) {
      AffineTransform3d affine = new AffineTransform3d ();
      fitRigid (affine, srcPnts, tgtPnts, scale);
      return affine;
   }
   
   public static AffineTransform3d fitRigid (MatrixNd srcPnts, 
      MatrixNd tgtPnts, Matrix K, boolean scale) {
      AffineTransform3d affine = new AffineTransform3d ();
      fitRigid (affine, srcPnts, tgtPnts, K, scale);
      return affine;
   }


   /**
    * 
    * @param srcPnts
    * @param tgtPnts
    * @param scale
    * @return
    */
   public static double fitRigid (AffineTransform3d affine, 
      MatrixNd srcPnts, MatrixNd tgtPnts, boolean scale) {

      if (srcPnts.rowSize () != tgtPnts.rowSize () ||
      srcPnts.colSize () != 3 || tgtPnts.colSize () != 3) {
         throw new ImproperSizeException ("Incompatible correspondence mapping size!");
      }

      MatrixNd Q = new MatrixNd ();
      MatrixNd P = new MatrixNd ();
      MatrixNd A = new MatrixNd(3, 3);
      Matrix3d U = new Matrix3d();
      Matrix3d V = new Matrix3d();
      Vector3d vec = new Vector3d();
      Vector3d qmean = new Vector3d();
      Vector3d pmean = new Vector3d();
      SVDecomposition svd = new SVDecomposition();
      AffineTransform3d bestX = new AffineTransform3d ();

      int n = srcPnts.rowSize ();

      VectorNd mean = new VectorNd ();
      PCA.centralizeData (Q, mean, srcPnts);
      qmean.set (mean);
      PCA.centralizeData (P, mean, tgtPnts);
      pmean.set (mean);

      double sq = Q.frobeniusNorm ();
      double sp = P.frobeniusNorm ();

      A.mulTransposeLeft(P, Q);
      A.scale(1.0 / n);
      svd.factor(A);
      svd.get(U, vec, V);

      double detU = U.orthogonalDeterminant();
      double detV = V.orthogonalDeterminant();
      if (detV * detU < 0) { /* then one is negative and the other positive */
         if (detV < 0) { /* negative last column of V */
            V.m02 = -V.m02;
            V.m12 = -V.m12;
            V.m22 = -V.m22;
            vec.z = -vec.z;
         }
         else /* detU < 0 */
         { /* negative last column of U */
            U.m02 = -U.m02;
            U.m12 = -U.m12;
            U.m22 = -U.m22;
            vec.z = -vec.z;
         }
      }

      bestX.A.mulTransposeRight(U, V);

      double s =  sp / sq;
      if (scale) {
         bestX.A.scale (s);
      }


      AffineTransform3d trans = new AffineTransform3d(bestX.A, new Vector3d(0,0,0));
      qmean.transform(trans);
      bestX.p.sub(pmean, qmean);
      
      // test
      /*
      MatrixNd testSrc = new MatrixNd  (srcPnts);
      testSrc.setSize (srcPnts.rowSize (), 4);
      VectorNd one = new VectorNd (testSrc.rowSize ());
      for (int i = 0; i < one.size ();i++) {
         one.set (i, 1);
      }
      testSrc.setColumn (3, one);
      testSrc.transpose ();
      testSrc.mul (bestX, testSrc);
      testSrc.transpose ();
      testSrc.setSize (testSrc.rowSize (), 3);
      MatrixNd testDis = new MatrixNd ();
      testDis.sub (srcPnts, tgtPnts);
      System.out.println ("old distatnce : " + testDis.frobeniusNorm ());
      testDis.sub (testSrc, tgtPnts);
      System.out.println ("new distatnce : " + testDis.frobeniusNorm ());
      VectorNd testvec = new VectorNd ();
      srcPnts.getRow (0, testvec);
      PrintData.printVector (testvec); */
      if (affine != null) {
         affine.set (bestX);
      }
      return s;
   }
   
   /**
    * 
    * @param srcPnts
    * @param tgtPnts
    * @param scale
    * @return
    */
   public static double fitRigid (AffineTransform3d affine, 
      MatrixNd srcPnts, MatrixNd tgtPnts, Matrix K, boolean scale) {

      if (srcPnts.rowSize () != tgtPnts.rowSize () ||
      srcPnts.colSize () != 3 || tgtPnts.colSize () != 3) {
         throw new ImproperSizeException ("Incompatible correspondence mapping size!");
      }

      MatrixNd Q = new MatrixNd ();
      MatrixNd P = new MatrixNd ();
      MatrixNd A = new MatrixNd(3, 3);
      Matrix3d U = new Matrix3d();
      Matrix3d V = new Matrix3d();
      Vector3d vec = new Vector3d();
      Vector3d qmean = new Vector3d();
      Vector3d pmean = new Vector3d();
      SVDecomposition svd = new SVDecomposition();
      AffineTransform3d bestX = new AffineTransform3d ();

      int n = srcPnts.rowSize ();

      VectorNd mean = new VectorNd ();
      PCA.centralizeData (Q, mean, srcPnts);
      qmean.set (mean);
      PCA.centralizeData (P, mean, tgtPnts);
      pmean.set (mean);
      
      VectorNd tgt = new VectorNd (tgtPnts.rowSize ()*3);
      for (int i = 0; i < tgtPnts.rowSize (); i++) {
         Point3d pos = new Point3d ();
         P.getRow (i, pos);
         tgt.setSubVector (i*3, pos);
      }
      VectorNd Ktgt = new VectorNd (tgt.size ());
      K.mul (Ktgt, tgt);
      MatrixNd KP  =null;
      try {
         KP = P.clone ();
      }
      catch (CloneNotSupportedException e) {
         e.printStackTrace();
      }
      for (int i = 0; i < tgtPnts.rowSize (); i++) {
         Point3d pos = new Point3d ();
         Ktgt.getSubVector (i*3, pos);
         KP.setRow (i, pos);
      }

      A.mulTransposeLeft(KP, Q);
      A.scale(1.0 / n);
      svd.factor(A);
      svd.get(U, vec, V);

      double detU = U.orthogonalDeterminant();
      double detV = V.orthogonalDeterminant();
      if (detV * detU < 0) { /* then one is negative and the other positive */
         if (detV < 0) { /* negative last column of V */
            V.m02 = -V.m02;
            V.m12 = -V.m12;
            V.m22 = -V.m22;
            vec.z = -vec.z;
         }
         else /* detU < 0 */
         { /* negative last column of U */
            U.m02 = -U.m02;
            U.m12 = -U.m12;
            U.m22 = -U.m22;
            vec.z = -vec.z;
         }
      }

      bestX.A.mulTransposeRight(U, V);

      VectorNd src = new VectorNd (srcPnts.rowSize ()*3);
      VectorNd Ksrc = new VectorNd (src.size ());
      for (int i = 0; i < Q.rowSize (); i++) {
         Point3d pos =  new Point3d ();
         Q.getRow (i, pos);
         Point3d rpos =  new Point3d ();
         rpos.mul (bestX.A, pos);
         Q.setRow (i, rpos);
         src.setSubVector (i*3, rpos);
      }
      K.mul (Ksrc, src);
      
      double sp = Ksrc.dot (src);
      double sq = Ksrc.dot (tgt);

      double s = 1;
      if (sq == 0) {
         s = 1;
      }
      else {
         s =  sp / sq;
      }

      if (scale) {
         bestX.A.scale (s);
      }

      AffineTransform3d trans = new AffineTransform3d(bestX.A, new Vector3d(0,0,0));
      qmean.transform(trans);
      bestX.p.sub(pmean, qmean);
      
      // test
      /*
      MatrixNd testSrc = new MatrixNd  (srcPnts);
      testSrc.setSize (srcPnts.rowSize (), 4);
      VectorNd one = new VectorNd (testSrc.rowSize ());
      for (int i = 0; i < one.size ();i++) {
         one.set (i, 1);
      }
      testSrc.setColumn (3, one);
      testSrc.transpose ();
      testSrc.mul (bestX, testSrc);
      testSrc.transpose ();
      testSrc.setSize (testSrc.rowSize (), 3);
      MatrixNd testDis = new MatrixNd ();
      testDis.sub (srcPnts, tgtPnts);
      System.out.println ("old distatnce : " + testDis.frobeniusNorm ());
      testDis.sub (testSrc, tgtPnts);
      System.out.println ("new distatnce : " + testDis.frobeniusNorm ());
      VectorNd testvec = new VectorNd ();
      srcPnts.getRow (0, testvec);
      PrintData.printVector (testvec); */
      if (affine != null) {
         affine.set (bestX);
      }
      return s;
   }

}
