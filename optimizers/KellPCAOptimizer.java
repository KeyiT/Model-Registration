package artisynth.models.swallowingRegistrationTool.optimizers;

import java.util.LinkedList;

import artisynth.models.swallowingRegistrationTool.utilities.PCA;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVTree;
import maspack.geometry.OBBTree;
import maspack.geometry.PointMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

/**
 * Reference : <i>Non-rigid transformations for Musculoskeletal Model, 
 * Petr Kellnhofer 2012 </i>;
 * 
 * @author KeyiTang
 *
 */
public class KellPCAOptimizer extends Optimizer{

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
      
      PointMapFormulator form = (PointMapFormulator) formulator;
      AffineSink asink = (AffineSink) sink;
      
      asink.takeOutput (
         globalAlignPCA(form.makeSourceDataMatrix (), 
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
      if (F instanceof PointMapFormulator) {
         return true;
      }
      return false;
   }



   public static AffineTransform3d globalAlignPCA (
      MatrixNd srcPnts, MatrixNd tgtPnts, boolean scale) {
      AffineTransform3d [] rawX = computeRawTransform (srcPnts, tgtPnts, scale);
      PointMesh srcMesh = new PointMesh ();
      PointMesh tgtMesh = new PointMesh ();
      double N = srcPnts.rowSize ();
      double M = tgtPnts.rowSize ();
      // target mesh
      for (int j = 0; j < M; j++) {
         Point3d pnt = new Point3d ();
         tgtPnts.getRow (j, pnt);
         tgtMesh.addVertex (pnt);
      }
      BVTree myTgtOBBTree = new OBBTree (tgtMesh, 2);

      BVFeatureQuery query = new BVFeatureQuery();
      int bestIdx = 0;
      double minDis = Double.MAX_VALUE;
      for (int i = 0; i < rawX.length; i++) {
         srcMesh.clear ();
         double dis = 0;
         // source mesh
         for (int j = 0; j < N; j++) {
            Point3d pnt = new Point3d ();
            srcPnts.getRow (j, pnt);
            pnt.transform (rawX[i]);
            srcMesh.addVertex (pnt);
            // measure
            // from src to tgt
            Vertex3d nearest = query.nearestVertexToPoint (myTgtOBBTree, pnt);
            dis += pnt.distance (nearest.pnt);
         }
         BVTree mySrcOBBTree = new OBBTree (srcMesh, 2);
         // measure
         // from tgt to src
         for (int j = 0; j < M; j++) {
            Point3d pnt = new Point3d ();
            pnt.set (tgtMesh.getVertex (j).pnt);
            Vertex3d nearest = query.nearestVertexToPoint (mySrcOBBTree, pnt);
            dis += pnt.distance (nearest.pnt);
         }

         // compare
         dis /= (M+N);
         System.out.println ("Distance is "+dis+".");
         if (dis < minDis) {
            bestIdx = i;
            minDis = dis;
         }

      }
      System.out.println ("Best Index: " + bestIdx);
      return rawX[bestIdx];
   }

   public static AffineTransform3d [] computeRawTransform(
      MatrixNd srcPnts, MatrixNd tgtPnts, boolean scale) {

      // radius
      // singular value
      // X1w
      // center
      Point3d [] srcBBV = new Point3d [8];
      Point3d [] tgtBBV = new Point3d [8];
      Vector3d srcSig = new Vector3d ();
      Vector3d tgtSig = new Vector3d ();
      RigidTransform3d srcX = PCA.computeBBVs (srcPnts, srcBBV, srcSig);
      RigidTransform3d tgtX = PCA.computeBBVs (tgtPnts, tgtBBV, tgtSig);

      Vector3d diaVec = new Vector3d ();
      diaVec.sub (srcBBV[7], srcBBV[0]);
      double srcRad = diaVec.maxElement ();
      diaVec.sub (tgtBBV[7], tgtBBV[0]);
      double tgtRad = diaVec.maxElement ();

      Vector3d srcCen = new Vector3d ();
      Vector3d tgtCen = new Vector3d ();
      srcCen.set (srcX.p);
      tgtCen.set (tgtX.p);


      RotationMatrix3d[] flips = computeFlips (srcSig);

      RigidTransform3d X21 = new RigidTransform3d();
      RigidTransform3d XRF = new RigidTransform3d();
      RotationMatrix3d[] Rots = new RotationMatrix3d[flips.length];

      for (int i=0; i<flips.length; i++) {
         // X21 = X1W * inv(X2W), so if we modify X2W to X2W' = X2W * XRF,
         // then
         // X21 = X1W * inv(XRF) * inv(X2W)
         XRF.R.set (flips[i]);
         X21.mulInverseRight (tgtX, XRF);
         X21.mulInverseRight (X21, srcX);
         //X21.mulInverseRight (myRefPCAF.myX1W, myTarPCAF.myX1W);
         Rots[i] = new RotationMatrix3d (X21.R);
      }
      AffineTransform3d [] Xlist = new AffineTransform3d [flips.length];
      for (int i=0; i<flips.length; i++) {
         Xlist[i] = new AffineTransform3d();
         if (scale) {
            computeTransform (Xlist[i], Rots[i], tgtCen, srcCen, tgtRad/srcRad);
         } 
         else {
            computeTransform (Xlist[i], Rots[i], tgtCen, srcCen, 1.0);
         }
         System.out.println (
            "det=" + Xlist[i].A.determinant());
      }

      return Xlist;
   }

   private static void addAxisFlips (
      LinkedList<RotationMatrix3d> flips,
      RotationMatrix3d R, Vector3d axis, int n) {

      RotationMatrix3d R0 = new RotationMatrix3d(R);
      flips.add (R0);
      double ang = 0;
      for (int i=1; i<n; i++) {
         ang += 2*Math.PI/n;
         RotationMatrix3d RX = new RotationMatrix3d(R);
         RX.mulAxisAngle (axis, ang);
         flips.add (RX);
      }
   }

   private static RotationMatrix3d[] computeFlips (Vector3d sig) {
      double sigTol = 1.5;
      LinkedList<RotationMatrix3d> flips = new LinkedList<RotationMatrix3d>();
      if (sig.x/sig.z < sigTol) {
         RotationMatrix3d R = new RotationMatrix3d();
         // try all 24 positions
         int ndivs = 4;
         addAxisFlips (flips, R, Vector3d.X_UNIT, ndivs);
         R.setAxisAngle (0, 1, 0, Math.PI/2);
         addAxisFlips (flips, R, Vector3d.X_UNIT, ndivs);
         R.setAxisAngle (0, 1, 0, Math.PI);
         addAxisFlips (flips, R, Vector3d.X_UNIT, ndivs);
         R.setAxisAngle (0, 1, 0, -Math.PI/2);
         addAxisFlips (flips, R, Vector3d.X_UNIT, ndivs);
         R.setAxisAngle (0, 0, 1, Math.PI/2);
         addAxisFlips (flips, R, Vector3d.X_UNIT, ndivs);
         R.setAxisAngle (0, 0, 1, -Math.PI/2);
         addAxisFlips (flips, R, Vector3d.X_UNIT, ndivs);
      }
      else if (sig.x/sig.y < sigTol) {
         RotationMatrix3d R = new RotationMatrix3d();
         addAxisFlips (flips, R, Vector3d.Z_UNIT, 4);
         R.setAxisAngle (0, 1, 0, Math.PI);
         addAxisFlips (flips, R, Vector3d.Z_UNIT, 4);
      }
      else if (sig.y/sig.z < sigTol) {
         RotationMatrix3d R = new RotationMatrix3d();
         addAxisFlips (flips, R, Vector3d.X_UNIT, 4);
         R.setAxisAngle (0, 1, 0, Math.PI);
         addAxisFlips (flips, R, Vector3d.X_UNIT, 4);
      }
      else {
         flips.add (RotationMatrix3d.IDENTITY);
         flips.add (new RotationMatrix3d (1, 0, 0, Math.PI));
         flips.add (new RotationMatrix3d (0, 1, 0, Math.PI));
         flips.add (new RotationMatrix3d (0, 0, 1, Math.PI));
      }
      return flips.toArray(new RotationMatrix3d[0]);
   }

   private static void computeTransform (
      AffineTransform3d X, RotationMatrix3d R,
      Vector3d c1, Vector3d c2, double scale) {

      X.A.set (R);
      X.p.transform (R, c2);
      X.p.scale (-scale);
      X.p.add (c1);
      X.A.scale (scale);
   }

}
