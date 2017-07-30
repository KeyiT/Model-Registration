package artisynth.models.swallowingRegistrationTool.BSI;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.correspondences.CloudMap;
import artisynth.models.swallowingRegistrationTool.correspondences.Correspondence;
import artisynth.models.swallowingRegistrationTool.correspondences.TrivariateInjectiveMap;
import artisynth.models.swallowingRegistrationTool.optimizers.GeneralizedPointInjectiveMapFormulator;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadLSFormulatorBase;
import artisynth.models.swallowingRegistrationTool.transformers.BlendLinear3dDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

/**
 * Sparse Quadratic least square formulator for 3D blending linear deformer
 * @author KeyiTang
 *
 */
public class SparseBLDTIMFormulator extends SparseQuadLSFormulatorBase{

   protected BlendLinear3dDeformer myDeformer;
   protected TrivariateInjectiveMap myMap;

   private VectorNd myWeights = new VectorNd ();
   private boolean weighted = false;

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      if (tf instanceof BlendLinear3dDeformer) {
         myDeformer = (BlendLinear3dDeformer)tf;
         return;
      }

      throw new ClassCastException ("Incompatible transformer!");
   }

   public BlendLinear3dDeformer getTransformer () {
      return myDeformer;
   }

   public TrivariateInjectiveMap getMap () {
      return myMap;
   }

   @Override
   public void setCloudMap (CloudMap map) {
      if (map instanceof TrivariateInjectiveMap) {
         myMap = (TrivariateInjectiveMap)map;
         return;
      }

      throw new ClassCastException ("Incompatible map!");
   }


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
       //TODO: test for time 
         System.out.println ("make basis...");
         Matrix Basis = myDeformer.makeBasis (
            myMap.makeSourceDataMatrix ());
         System.out.println ("make basis done!");

         if (Basis == null) {
            return false;
         }

         A.set (Basis);

         MatrixNd tmp = myMap.makeTargetedChangeMatrix ();

         b.setSize (tmp.colSize () * tmp.rowSize ());
         int idx = 0;
         for (int i = 0; i < tmp.rowSize (); i++) {
            for (int j = 0; j < tmp.colSize (); j++) {
               b.set (idx++, tmp.get (i, j));
            }
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
            //TODO: test for time 
            System.out.println ("make basis...");
            Matrix Basis = myDeformer.makeBasis (
               myMap.makeSourceDataMatrix ());
            System.out.println ("make basis done!");
            if (Basis == null) {
               return false;
            }
            
            MatrixNd tmp = myMap.makeTargetedChangeMatrix ();
            VectorNd b = new VectorNd ();
            b.setSize (tmp.colSize () * tmp.rowSize ());
            int idx = 0;
            for (int i = 0; i < tmp.rowSize (); i++) {
               for (int j = 0; j < tmp.colSize (); j++) {
                  b.set (idx++, -tmp.get (i, j));
               }
            }
            
            applyMapWeights ((SparseBlockMatrix)Basis, b);
            
            SparseBlockMatrix CovTgt = (SparseBlockMatrix)
            ((GeneralizedPointInjectiveMapFormulator)myMap).createTargetGeneralizeMatrix ();
            SparseBlockMatrix BC = new SparseBlockMatrix ();
            BC.mulTransposeLeftToBlkMat ((SparseBlockMatrix)Basis, CovTgt);
            
            // quadratic term
            rH.mulToBlkMat (BC, (SparseBlockMatrix) Basis);
            
            // proportional term
            BC.mul (rq, b);
            
            // constant term
            VectorNd CDis = new VectorNd ();
            CovTgt.mul (CDis, b);
            double c = CDis.dot (b);
            addConstantTerm (c);
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


}