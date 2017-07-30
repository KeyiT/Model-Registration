package artisynth.models.swallowingRegistrationTool.BSI;

import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.optimizers.GeneralizedPointInjectiveMapFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPoint3d;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.ImproperStateException;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class NFFDTIMFormulator extends SparseBLDTIMFormulator{
   
   NFFDeformer myDeformer;

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      
      if (tf instanceof NFFDeformer) {
         myDeformer = (NFFDeformer) tf;
      }
      else {
         throw new ClassCastException ("Incompatible transformer!");
      }

      super.setTransformer (tf);
   }

   @Override
   public NFFDeformer getTransformer () {
      return myDeformer;
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
         Map<Vector3d, Vector3d> map = myMap.getInjectiveMap ();
         if (map.size () == 0) {
            return false;
         }
         SparseBlockMatrix Basis = makeInteralBasis (map);
         System.out.println ("make basis done");
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
   
   protected SparseBlockMatrix makeInteralBasis (Map<Vector3d, Vector3d> map) {
      Set ens = map.entrySet ();
      Iterator it = ens.iterator ();
      SparseBlockMatrix B = new SparseBlockMatrix ();
      
      while (it.hasNext ()) {
         Entry<Vector3d, Vector3d> me = 
         (Entry<Vector3d, Vector3d>)it.next ();
         
         Vector3d key = me.getKey ();
         if (key instanceof Point3d) {
            Point3d pnt = (Point3d)key;
            NFFDPoint3d ffdPnt = 
            myDeformer.getFFD ().getNFFDPoint (pnt);
            if (ffdPnt != null) {
               SparseBlockMatrix basis = ffdPnt.createBasisMatrix ();
               addBlockMatrixByRow (B, basis);
            }
            else {
               SparseBlockMatrix basis = myDeformer.makeBasis (pnt);
               addBlockMatrixByRow (B, basis);
            }
         }
         else {
            SparseBlockMatrix basis = 
            myDeformer.makeBasis (new Point3d (key));
            addBlockMatrixByRow (B, basis);
         }
      }
      
      return B;
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
            Map<Vector3d, Vector3d> map = myMap.getInjectiveMap ();
            if (map.size () == 0) {
               return false;
            }
            SparseBlockMatrix Basis = makeInteralBasis (map);
            System.out.println ("make basis done!");
            
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
   
   

}
