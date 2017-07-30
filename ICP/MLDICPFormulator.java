package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.List;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.CloudMap;
import artisynth.models.swallowingRegistrationTool.correspondences.Correspondence;
import artisynth.models.swallowingRegistrationTool.correspondences.VertexToMeshCorrespondence;
import artisynth.models.swallowingRegistrationTool.optimizers.GeneralizedPointInjectiveMapFormulator;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadLSFormulatorBase;
import artisynth.models.swallowingRegistrationTool.transformers.MeshBasedLinearDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import artisynth.models.swallowingRegistrationTool.utilities.PCA;
import maspack.geometry.Face;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.VectorNd;
import maspack.util.InternalErrorException;

public class MLDICPFormulator extends SparseQuadLSFormulatorBase{

   private MeshBasedLinearDeformer myDeformer;
   private ClosestPoint3dMap<? extends VertexToMeshCorrespondence> myMap;

   private VectorNd myWeights = new VectorNd ();
   private boolean weighted = false;
   
   private boolean normalize = true;

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      if (tf instanceof MeshBasedLinearDeformer) {
         myDeformer = (MeshBasedLinearDeformer)tf;
         return;
      }

      throw new ClassCastException ("Incompatible transformer!");
   }

   public MeshBasedLinearDeformer getTransformer () {
      return myDeformer;
   }

   public ClosestPoint3dMap getMap () {
      return myMap;
   }

   @Override
   public void setCloudMap (CloudMap map) {
      if (map instanceof ClosestPoint3dMap) {
         myMap = (ClosestPoint3dMap)map;
         return;
      }

      throw new ClassCastException ("Incompatible map!");
   }



   /**
    * {@inheritDoc}
    */
   @Override
   public boolean updateInternalGoals (SparseBlockMatrix A, VectorNd b) {
      if (myMap == null) {
         throw new ImproperStateException ("Map not initialized");
      }
      if (myDeformer == null) {
         throw new ImproperStateException ("Deformer not initialized");
      }

      if (myMap instanceof GeneralizedPointInjectiveMapFormulator) {
         return true;
      }

      try {
         int [] bs = new int [myDeformer.getSize ()/3];
         int [] rbs = new int [myMap.numInjectiveMaps ()];

         for (int i = 0; i < rbs.length; i++) {
            rbs[i] = 3;
         }
         for (int i = 0; i < bs.length; i++) {
            bs[i] = 3;
         }
         SparseBlockMatrix K = new SparseBlockMatrix (rbs, bs);
         VectorNd tgt = new VectorNd (rbs.length * 3);

         List<Vertex3d> vts = myDeformer.collectVertices ();
         int mapIdx = 0;
         for (VertexToMeshCorrespondence corr : myMap.getInfos ()) {

            if (corr.getDirection ()) {
               // src --> target
               if (!myMap.isSourceToTargetEnabled ()) {
                  continue;
               }
               Vertex3d vtx = corr.getVertex ();
               int idx = vts.indexOf (vtx);
               if (idx == -1) {
                  throw new InternalErrorException ("");
               }
               Matrix3x3Block blk = (Matrix3x3Block)K.getBlock (mapIdx, idx);
               if (blk == null) {
                  blk = new Matrix3x3Block();
                  blk.setIdentity ();
                  K.addBlock (mapIdx, idx, blk);
               }
               
               Point3d near = corr.getNearestPoint ();
               tgt.setSubVector (mapIdx * 3, near);
               mapIdx ++;
            }
            else {
               // target --> source
               if (!myMap.isTargetToSourceEnabled ()) {
                  continue;
               }
               Face face = (Face)corr.getNearestFeature ();

               Vector2d bar = corr.getBarycentric ();
               Vertex3d [] vts3 = face.getTriVertices ();
               for (int i = 0; i < 3; i++) {
                  int idx = vts.indexOf (vts3[i]);
                  if (idx == -1) {
                     throw new InternalErrorException ("");
                  }
                  Matrix3x3Block blk = (Matrix3x3Block)K.getBlock (mapIdx, idx);
                  if (blk == null) {
                     blk = new Matrix3x3Block();
                     blk.setIdentity ();
                     K.addBlock (mapIdx, idx, blk);
                  }
                  double val = 0;
                  if (i > 0) {
                     val = bar.get (i-1);
                  }
                  else {
                     val = 1 - bar.x - bar.y;
                  }
                  blk.m00 = val;
                  blk.m11 = val;
                  blk.m22 = val;
               }

               Point3d near = new Point3d ();   
               corr.getVertex ().getWorldPoint (near);
               tgt.setSubVector (mapIdx*3, near);
               mapIdx++;
            }
         }
         A.set (K);
         b.set (tgt);
         if (normalize) {
            MatrixNd tgtData = myMap.makeTargetDataMatrix ();
            MatrixNd Cov = new MatrixNd();
            PCA.computeCov (Cov, tgtData);
            double dev = Cov.get (0, 0) + Cov.get (1, 1) + Cov.get (2, 2);
            dev = Math.sqrt (dev);
            A.scale (1.0/ dev / Math.sqrt ((double)myMap.numInjectiveMaps ()));
            b.scale (2.0/ dev / Math.sqrt ((double)myMap.numInjectiveMaps ()));
         }
         else {
            b.scale (2.0);
         }
         applyMapWeights (A, b);
         
      }
      catch (Exception e) {
         e.printStackTrace ();
         return false;
      }

      return true;
   }


   @Override
   protected boolean getInternalTerm (SparseBlockMatrix rH, VectorNd rq) {
      if (myMap == null) {
         throw new ImproperStateException ("Map not initialized");
      }
      if (myDeformer == null) {
         throw new ImproperStateException ("Deformer not initialized");
      }

      if (myMap instanceof GeneralizedPointInjectiveMapFormulator) {
         try {
            int [] bs = new int [myDeformer.getSize ()/3];
            int [] rbs = new int [myMap.numInjectiveMaps ()];

            for (int i = 0; i < rbs.length; i++) {
               rbs[i] = 3;
            }
            for (int i = 0; i < bs.length; i++) {
               bs[i] = 3;
            }
            SparseBlockMatrix K = new SparseBlockMatrix (rbs, bs);
            VectorNd tgt = new VectorNd (rbs.length * 3);

            List<Vertex3d> vts = myDeformer.collectVertices ();
            int mapIdx = 0;
            for (VertexToMeshCorrespondence corr : myMap.getInfos ()) {

               if (corr.getDirection ()) {
                  // src --> target
                  if (!myMap.isSourceToTargetEnabled ()) {
                     continue;
                  }
                  Vertex3d vtx = corr.getVertex ();
                  int idx = vts.indexOf (vtx);
                  if (idx == -1) {
                     throw new InternalErrorException ("");
                  }
                  Matrix3x3Block blk = (Matrix3x3Block)K.getBlock (mapIdx, idx);
                  if (blk == null) {
                     blk = new Matrix3x3Block();
                     K.addBlock (mapIdx, idx, blk);
                  }

                  blk.m00 += 1;
                  blk.m11 += 1;
                  blk.m22 += 1;

                  Point3d near = corr.getNearestPoint ();
                  tgt.setSubVector (mapIdx * 3, near);
                  mapIdx ++;
               }
               else {
                  // target --> source
                  if (!myMap.isTargetToSourceEnabled ()) {
                     continue;
                  }
                  Face face = (Face)corr.getNearestFeature ();

                  Vector2d bar = corr.getBarycentric ();
                  Vertex3d [] vts3 = face.getTriVertices ();
                  for (int i = 0; i < 3; i++) {
                     int idx = vts.indexOf (vts3[i]);
                     if (idx == -1) {
                        throw new InternalErrorException ("");
                     }
                     Matrix3x3Block blk = (Matrix3x3Block)K.getBlock (mapIdx, idx);
                     if (blk == null) {
                        blk = new Matrix3x3Block();
                        K.addBlock (mapIdx, idx, blk);
                     }
                     double val = 0;
                     if (i > 0) {
                        val = bar.get (i-1);
                     }
                     else {
                        val = 1 - bar.x - bar.y;
                     }

                     blk.m00 += val;
                     blk.m11 += val;
                     blk.m22 += val;
                  }

                  Point3d near = new Point3d ();   
                  corr.getVertex ().getWorldPoint (near);
                  tgt.setSubVector (mapIdx*3, near);
                  mapIdx++;
               }
            }

            applyMapWeights (K, tgt);

            SparseBlockMatrix CovTgt = (SparseBlockMatrix)
            ((GeneralizedPointInjectiveMapFormulator)myMap).createTargetGeneralizeMatrix ();
            SparseBlockMatrix KC = new SparseBlockMatrix ();
            KC.mulTransposeLeftToBlkMat (K, CovTgt);

            // quadratic term
            rH.mulToBlkMat (KC, (SparseBlockMatrix) K);

            // proportional term
            KC.mul (rq, tgt);
            
            if (normalize) {
               MatrixNd tgtData = myMap.makeTargetDataMatrix ();
               MatrixNd Cov = new MatrixNd();
               PCA.computeCov (Cov, tgtData);
               double dev = Cov.get (0, 0) + Cov.get (1, 1) + Cov.get (2, 2);
               rH.scale (1.0/ dev / (double)myMap.numInjectiveMaps ());
               rq.scale (-2.0/ dev / (double)myMap.numInjectiveMaps ());
            }
            else {
               rq.scale (-2.0);
            }
            
         }
         catch (Exception e) {
            e.printStackTrace ();
            return false;
         }
      }

      return true;
   }

   protected void applyMapWeights (SparseBlockMatrix A, VectorNd b) {
      if (weighted) {
         if (myMap.numInfos () != myWeights.size ()) {
            throw new ImproperSizeException ("Incompatible weights!");
         }

         int idx = 0;
         for (int i = 0; i < myMap.numInfos (); i ++) {
            Correspondence corr = (Correspondence)myMap.getInfo (i);
            if (corr.isMuted ()) {
               continue;
            }

            double w = myWeights.get (i);
            w = Math.sqrt (Math.abs (w));
            for (int j = 0; j < 3; j++) {
               idx = i*3 + j;
               b.set (idx, b.get (idx) * w);
            }
            MatrixBlock blk = A.firstBlockInRow (i);
            while (blk != null) {
               blk.scale (w);
               blk = blk.next ();
            }
         }
      }
   }

   /**
    * Any subclass should assign a weight for each map. 
    * 
    * This method is to retrieve the mapping weights <tt>W_n</tt> which is defined 
    * as a (<tt>N</tt>) elements vector. 
    * 
    * @return mapping weights matrix <tt>W_n</tt>.
    */
   public VectorNd getMapWeights () {
      return myWeights;
   }

   /**
    * enable mapping weight;
    * @param enable if false every map is treated in the same manner
    */
   public void enableMapWeight (boolean enable) {
      weighted = enable;
   }

   /**
    * @return if different correspondences have different 
    * weight return true, otherwise return false
    */
   public boolean mapWeightEnabled () {
      return weighted;
   }

   /**
    * set weight vector size to zero
    */
   public void clearMapWeights () {
      myWeights.setSize (0);
   }

   /**
    * 
    */
   public void setMapWeights (VectorNd weights) {
      throw new UnsupportedOperationException("");
   }
   
   public void enableNormalize (boolean enable) {
      normalize = enable;
   }

   public boolean isNormalized () {
      return normalize;
   }

}
