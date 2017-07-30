package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.HashMap;
import java.util.Map;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.utilities.GeometryOperator;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.ScaledRigidTransform3d;
import maspack.matrix.VectorNd;

public class ACAPDeformer extends MeshBasedLinearDeformer{

   Map <PolygonalMesh, SparseBlockMatrix> myLs = 
   new HashMap <PolygonalMesh, SparseBlockMatrix> ();
   Map <PolygonalMesh, double []> myVoronoi = 
   new HashMap <PolygonalMesh, double []> ();

   public ACAPDeformer () {
      super ();
   }

   @Override
   public SparseBlockMatrix createStiffnessMatrix () {
      return createStiffnessMatrix (false);
   }

   public SparseBlockMatrix createStiffnessMatrix (boolean normalize) {
      int bs [] = new int [getSize()/3];
      for (int i = 0; i < bs.length; i++) {
         bs [i]  = 3;
      }

      SparseBlockMatrix K = 
      new SparseBlockMatrix  (bs, bs);

      for (int meshIdx = 0; meshIdx < myMeshes.size (); meshIdx ++) {
         PolygonalMesh mesh = myMeshes.get (meshIdx);
         if (myLs.get (mesh) == null) {
            collectLaplacians ();
         }

         double [] areas = myVoronoi.get (mesh).clone ();
         SparseBlockMatrix Ls = myLs.get (mesh);
         
         HashMap <Point3d, Point3d> pointMap;
         pointMap = creatPointMap ();
         
         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            ScaledRigidTransform3d map = GeometryOperator.
            computeCellLocalConformalMap (mesh.getVertex (vtxIdx), 
               pointMap, myLs.get (mesh));
            
            // TODO
            //areas [vtxIdx] = areas [vtxIdx] * map.s * map.s;
         }

         // normalize
         int numEdge2 = 0;
         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            Matrix3x3Block blk = 
            (Matrix3x3Block)Ls.firstBlockInRow (vtxIdx);
            while (blk != null) {
               int colIdx = blk.getBlockCol ();
               if (colIdx == vtxIdx) {
                  blk = (Matrix3x3Block)blk.next ();
                  continue;
               }
               blk = (Matrix3x3Block)blk.next ();
               numEdge2 ++;
            }
         }
         double s = 0;
         if (normalize) {
            for (double area: areas) {
               s += area;
            }
            s = Math.sqrt (s);
            s /= (double)numEdge2;
         }


         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            int KIdx_i = getKIdx (meshIdx, vtxIdx);
            double k_ii = 0;

            Matrix3x3Block blk = 
            (Matrix3x3Block)Ls.firstBlockInRow (vtxIdx);
            while (blk != null) {
               int colIdx = blk.getBlockCol ();
               if (colIdx == vtxIdx) {
                  blk = (Matrix3x3Block)blk.next ();
                  continue;
               }
               double w_ij = blk.m00;
               double k_ij = w_ij * w_ij *
               (1.0/areas[vtxIdx] + 1.0/areas[colIdx]);
               // accumulate
               k_ii += k_ij;
               // 
               k_ij = - k_ij;
               // block for K
               Matrix3x3Block KBlk = new Matrix3x3Block ();
               KBlk.setDiagonal (k_ij, k_ij, k_ij);
               K.addBlock (KIdx_i, 
                  getKIdx (meshIdx, colIdx), KBlk);
               if (normalize) {
                  KBlk.scale (s);
               }
               blk = (Matrix3x3Block)blk.next ();
               numEdge2 ++;
            }

            // block for K
            Matrix3x3Block KBlk = new Matrix3x3Block ();
            KBlk.setDiagonal (k_ii, k_ii, k_ii);
            K.addBlock (KIdx_i, KIdx_i, KBlk);
            if (normalize) {
               KBlk.scale (s);
            }
         }
      }

      myK = K;

      return K;
   }



   @Override
   public VectorNd creatTargetForce () {
      return creatTargetForce (false);
   }

   public VectorNd creatTargetForce (boolean normalize) {
      int bs [] = new int [getSize()/3];
      for (int i = 0; i < bs.length; i++) {
         bs [i]  = 3;
      }

      SparseBlockMatrix K = 
      new SparseBlockMatrix  (bs, bs);

      // ---evaluate local rotations for each cell--- //
      HashMap <Vertex3d, Matrix3d> RotMap = 
      new HashMap <Vertex3d, Matrix3d> ();

      HashMap <Point3d, Point3d> pointMap;
      pointMap = creatPointMap ();
      for (PolygonalMesh mesh: myMeshes) {
         int vtxIdx = 0;
         double [] areas = myVoronoi.get (mesh);
         for (Vertex3d vtx : mesh.getVertices ()) {
            ScaledRigidTransform3d map = GeometryOperator.
            computeCellLocalConformalMap (vtx, pointMap, myLs.get (mesh));

            Matrix3d CellR = new Matrix3d (map.R);
            // TODO
            CellR.scale (1.0 / areas[vtxIdx] * map.s);
            vtxIdx ++;
            RotMap.put (vtx, CellR);
         }
      }


      for (int meshIdx = 0; meshIdx < myMeshes.size (); meshIdx ++) {
         PolygonalMesh mesh = myMeshes.get (meshIdx);
         if (myLs.get (mesh) == null) {
            collectLaplacians ();
         }

         SparseBlockMatrix Ls = myLs.get (mesh);
         double [] areas = myVoronoi.get (mesh);

         // normalize
         int numEdge2 = 0;
         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            Matrix3x3Block blk = 
            (Matrix3x3Block)Ls.firstBlockInRow (vtxIdx);
            while (blk != null) {
               int colIdx = blk.getBlockCol ();
               if (colIdx == vtxIdx) {
                  blk = (Matrix3x3Block)blk.next ();
                  continue;
               }
               blk = (Matrix3x3Block)blk.next ();
               numEdge2 ++;
            }
         }
         double s = 0;
         if (normalize) {
            for (double area: areas) {
               s += area;
            }
            s = Math.sqrt (s);
            s /= (double)numEdge2;
         }

         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            int KIdx_i = getKIdx (meshIdx, vtxIdx);

            Matrix3x3Block R_ii=  new Matrix3x3Block();

            Matrix3x3Block blk = 
            (Matrix3x3Block)Ls.firstBlockInRow (vtxIdx);
            while (blk != null) {
               int colIdx = blk.getBlockCol ();
               if (colIdx == vtxIdx) {
                  blk = (Matrix3x3Block)blk.next ();
                  continue;
               }
               double w_ij = blk.m00;
               Matrix3d Ri = RotMap.get (
                  mesh.getVertex (vtxIdx));
               Matrix3d Rj = RotMap.get (
                  mesh.getVertex (colIdx));

               Matrix3d R_ij = new Matrix3d ();
               R_ij.add (Rj, Ri);
               R_ij.scale (w_ij*w_ij);
               //accumulate
               R_ii.add (R_ij);
               //
               R_ij.negate ();
               // block for K
               Matrix3x3Block KBlk = new Matrix3x3Block ();
               KBlk.set (R_ij);
               K.addBlock (KIdx_i, 
                  getKIdx (meshIdx, colIdx), KBlk);
               if (normalize) {
                  KBlk.scale (s);
               }
               blk = (Matrix3x3Block)blk.next ();
            }

            // block for K
            Matrix3x3Block KBlk = new Matrix3x3Block ();
            KBlk.set (R_ii);
            if (normalize) {
               KBlk.scale (s);
            }
            K.addBlock (KIdx_i, KIdx_i, KBlk);
         }
      }

      VectorNd F = new VectorNd (getSize());
      K.mul (F, myRestPosition);

      return F;
   }



   private void collectLaplacians () {
      for (PolygonalMesh mesh : myMeshes) {
         SparseBlockMatrix Ls = 
         GeometryOperator.createLaplacianMapMatrix (
            mesh, false);
         myLs.put (mesh, Ls);

         // voronoi area
         double [] areas = new double [mesh.numVertices ()];
         int idx = 0;
         for (Vertex3d vtx : mesh.getVertices ()) {
            double area = GeometryOperator.
            computeVoronoiArea (vtx);
            areas [idx++] = area;
         }

         myVoronoi.put (mesh, areas);
      }
   }

   public void advanceRigidModes () {
      int meshIdx = 0;
      for (PolygonalMesh mesh : myMeshes) {
         MatrixNd srcData = new MatrixNd (mesh.numVertices (), 3);
         MatrixNd tgtData = new MatrixNd (mesh.numVertices (), 3);

         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            int idx = this.getKIdx (meshIdx, vtxIdx);
            Point3d pnt = new Point3d ();
            myPosition.getSubVector (idx*3, pnt);
            tgtData.setRow (vtxIdx, pnt);
            myRestPosition.getSubVector (idx*3, pnt);
            srcData.setRow (vtxIdx, pnt);
         }

         AffineTransform3d affine = new AffineTransform3d ();
         double s = ARUNSVDOptimizer.fitRigid (affine, srcData, tgtData, true);

         double [] areas = myVoronoi.get (mesh);

         for (int vtxIdx = 0; vtxIdx < mesh.numVertices (); vtxIdx++) {
            int idx = this.getKIdx (meshIdx, vtxIdx);
            Point3d pnt = new Point3d ();
            myRestPosition.getSubVector (idx*3, pnt);
            pnt.transform (affine);
            myRestPosition.setSubVector (idx*3, pnt);
         }

         if (areas != null) {
            for (int i = 0; i < areas.length; i++) {
               areas[i] *= s * s;
            }
         }

         meshIdx++;
      }
   }

}
