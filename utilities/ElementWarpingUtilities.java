package artisynth.models.swallowingRegistrationTool.utilities;

import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.ScaledRigidTransform3d;

/**
 * 
 * @author KeyiTang
 *
 */
public class ElementWarpingUtilities {
   
   public static void evalWarpingRotation (ScaledRigidTransform3d Re, 
      FemElement3d ele, MatrixNd restData) {
      
      if (restData.rowSize () != ele.numNodes () ||
          restData.colSize () != 3) {
         throw new IllegalArgumentException ("Incompatible matrix size");
      }
      MatrixNd currentData = new MatrixNd (ele.numNodes (), 3);
      int idx = 0;
      for (FemNode3d node : ele.getNodes ()) {
         currentData.setRow (idx++, node.getPosition ());
      }
      
      
      AffineTransform3d affine = new AffineTransform3d ();
      double scale = ARUNSVDOptimizer.fitRigid (affine, 
         restData, currentData, false);
      
      Re.R.set (affine.A);
      Re.p.set (affine.p);
      Re.setScale (scale);
   }

   public static double evalWarpingRotation (RotationMatrix3d Re, 
      FemElement3d ele, MatrixNd restData) {
      
      if (restData.rowSize () != ele.numNodes () ||
          restData.colSize () != 3) {
         throw new IllegalArgumentException ("Incompatible matrix size");
      }
      MatrixNd currentData = new MatrixNd (ele.numNodes (), 3);
      int idx = 0;
      for (FemNode3d node : ele.getNodes ()) {
         currentData.setRow (idx++, node.getPosition ());
      }
      
      
      AffineTransform3d affine = new AffineTransform3d ();
      double scale = ARUNSVDOptimizer.fitRigid (affine, 
         restData, currentData, false);
      
      Re.set (affine.A);
      return scale;
   }
   
   
   
   public static double evalWarpingRotation (RotationMatrix3d Re, 
      FemElement3d ele) {
      
      MatrixNd restData = new MatrixNd (ele.numNodes (), 3);
      int idx = 0;
      for (FemNode3d node : ele.getNodes ()) {
         restData.setRow (idx++, node.getRestPosition ());
      }
      
      double scale = evalWarpingRotation (Re, ele, restData);
      return scale;
   }
   
   public static void evalWarpingRotation (ScaledRigidTransform3d Re, 
      FemElement3d ele) {
      
      MatrixNd restData = new MatrixNd (ele.numNodes (), 3);
      int idx = 0;
      for (FemNode3d node : ele.getNodes ()) {
         restData.setRow (idx++, node.getRestPosition ());
      }
      
      evalWarpingRotation (Re, ele, restData);
   }
   
   public static double evalWarpingRotation (RotationMatrix3d Re, 
      FemNode3d node) {
      
      if (node.numAdjacentElements () == 0) {
         throw new IllegalArgumentException (
            "Isolated node!");
      }
      
      LinkedList<FemElement3d> eles = node.getElementDependencies ();
      Iterator it = eles.iterator ();
      Set<FemNode3d> nodes = new HashSet<FemNode3d> ();
      while (it.hasNext ()) {
         FemElement3d ele = (FemElement3d)it.next ();
         for (FemNode3d no : ele.getNodes ()) {
            if (!nodes.contains (no)) {
               nodes.add (no);
            }
         }
      }
      
      int numNode = nodes.size ();
      MatrixNd currentData = new MatrixNd (numNode, 3);
      MatrixNd restData = new MatrixNd (numNode, 3);
      
      it = nodes.iterator ();
      int idx = 0;
      
      while (it.hasNext ()) {
         FemNode3d no = (FemNode3d)it.next ();
         currentData.setRow (idx, no.getPosition ());
         restData.setRow (idx, no.getRestPosition ());
         idx++;
      }
      
      AffineTransform3d affine = new AffineTransform3d ();
      double scale = ARUNSVDOptimizer.fitRigid (affine, 
         restData, currentData, false);
      
      //For test
      /*
      System.out.println ("determinant of R: "+ affine.A.determinant ());
      MatrixNd trans = new MatrixNd (restData.rowSize (), 3);
      for (int i = 0; i < trans.rowSize (); i++) {
         trans.setRow (i, affine.p);
      }
      MatrixNd cD = new MatrixNd ();
      MatrixNd rD = new MatrixNd ();
      PCA.centralizeData (cD, currentData);
      PCA.centralizeData (rD, restData);
      MatrixNd disData = new MatrixNd ();
      disData.sub (cD, rD);
      System.out.println ("old distance: " + disData.frobeniusNorm ());
      rD.mulTranspose (new MatrixNd(affine.A));
      disData.sub (rD, cD);
      System.out.println ("new distance: " + disData.frobeniusNorm ());*/
      
      
      Re.set (affine.A);
      return scale;
   }
   
   public static SparseBlockMatrix assembleWarpingMatrix (
      RotationMatrix3d Re, FemElement3d ele) {
      
      int [] rbs = new int [ele.numNodes ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix ReMat = new SparseBlockMatrix (rbs, rbs);
      ReMat.setVerticallyLinked (true);
      Matrix3x3Block blk;
      for (int i = 0; i < rbs.length; i++) {
         blk = new Matrix3x3Block ();
         blk.set (Re);
         ReMat.addBlock (i, i, blk.clone ());
      }
      
      return ReMat;
   }
   
   public static SparseBlockMatrix assembleWarpingMatrix (
      RotationMatrix3d Re, int numNodes) {
      
      int [] rbs = new int [numNodes];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix ReMat = new SparseBlockMatrix (rbs, rbs);
      ReMat.setVerticallyLinked (true);
      Matrix3x3Block blk;
      for (int i = 0; i < rbs.length; i++) {
         blk = new Matrix3x3Block ();
         blk.set (Re);
         ReMat.addBlock (i, i, blk.clone ());
      }
      
      return ReMat;
   }
   
   /**
    *  Kr = Re eleK R^T
    * @param Re
    * @param eleK
    */
   public static void warpStiffnessMatrix (
      SparseBlockMatrix eleK, RotationMatrix3d Re) {
      SparseBlockMatrix RR = assembleWarpingMatrix (
         Re, eleK.numBlockRows ());
      
      SparseBlockMatrix Kr = new SparseBlockMatrix ();
      Kr.mulTransposeRightToBlkMat (eleK, RR);
      eleK.mulToBlkMat (RR, Kr);
   }
   
   public static void evalScaledRigidTransform (ScaledRigidTransform3d R, FemModel3d fem) {
     Set <FemNode3d> nodes = new HashSet<FemNode3d> ();
     
     for (FemNode3d node : fem.getNodes ()) {
        if (node.numAdjacentElements () != 0) {
           nodes.add (node);
        }
     }
     
     MatrixNd cuData = new MatrixNd (nodes.size (), 3);
     MatrixNd reData = new MatrixNd (nodes.size (), 3);
     
     int idx = 0;
     for (FemNode3d node : nodes) {
        cuData.setRow(idx, node.getPosition ());
        reData.setRow (idx, node.getRestPosition ());
        idx++;
     }
     
     AffineTransform3d affine = new AffineTransform3d ();
     double scale = ARUNSVDOptimizer.fitRigid (affine, reData, cuData, false);
     R.setTranslation (affine.p);
     R.R.set (affine.A);
     R.setScale (scale);
   }

}
