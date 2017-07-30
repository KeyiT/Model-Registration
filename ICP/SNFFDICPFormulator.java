package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.ArrayList;
import java.util.List;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.correspondences.TrivariateInjectiveMap;
import artisynth.models.swallowingRegistrationTool.transformers.BlendLinear3dDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.FFDPointNotFoundException;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPoint3d;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public class SNFFDICPFormulator extends SBLDICPFormulator{

   private NFFDeformer myDeformer;

   private MatrixNd myIntSrc;
   private MatrixNd myIntTgt;
   private int [][] myIntSubdomains;
   
   private List<NFFDPoint3d> mySrcPnts = new ArrayList<NFFDPoint3d> ();


   @Override
   public void setTransformer (RegistrationTransformer tf) {

      super.setTransformer (tf);

      if (tf instanceof NFFDeformer) {
         myDeformer = (NFFDeformer)tf;
         return;
      }
      
      throw new UnsupportedOperationException (
         "Incompatible transformer!");
   }

   public BlendLinear3dDeformer getTransformer () {
      if (myDeformer == null) {
         return super.getTransformer ();
      }
      return (BlendLinear3dDeformer)myDeformer;
   }

   public int getWorkingSubdomain () {
      return myWorkDomain;
   }

   @Override
   public boolean updateInternalGoals (SparseBlockMatrix A, VectorNd b) {
      if (myDeformer == null) {
         return super.updateInternalGoals (A, b);
      }

      if (! myDeformer.isSubdivisionEnabled ()) {
         return super.updateInternalGoals (A, b);
      }

      if (getMap () == null) {
         throw new ImproperStateException ("Map not initialized");
      }
      if (myDeformer == null) {
         throw new ImproperStateException ("Deformer not initialized");
      }

      TrivariateInjectiveMap myMap = getMap ();

      try {
         if (!dataValidP) {
            mySrcPnts.clear ();
            
            myIntSrc = myMap.makeSourceDataMatrix ();
            myIntTgt = myMap.makeTargetedChangeMatrix ();

            for (int i = 0; i < myIntSrc.rowSize (); i++) {
               Point3d pnt = new Point3d ();
               myIntSrc.getRow (i, pnt);
               
               NFFDPoint3d fp = new NFFDPoint3d ();
               fp.setFFD (myDeformer.getFFD ());
               fp.setUndeformedPosition (pnt);
               
               try {
                  fp.evalUVT ();
               }
               catch (FFDPointNotFoundException e) {
                  System.err.println (
                  "Failed to find sub-domains for source points!");
               return false;
               }
               
               mySrcPnts.add (fp);
            }

            dataValidP = true;
         }

         // test
         /*
         System.out.println ("data subdomains: "); 
         for (int [] dos : myIntSubdomains) {
            for (int doi : dos) {
               System.out.print (doi + " ");
            }
            System.out.println ("");
         }*/


         myWorkDomain = myDeformer.getWorkingSubdomain ();
         MatrixNd tgt = new MatrixNd ();
         
         A.set (getSrcPntsSubdomainBasis ());
         if (A.rowSize () == 0) {
            b.setSize (0);
            return true;
         }

         int num = 0;
         for (int i = 0; i < myIntTgt.rowSize (); i++) {
            if (myIntMarks[i]) {
               tgt.setSize (num+1, 3);
               for (int j = 0; j < 3; j++) {
                  tgt.set (num, j, myIntTgt.get (i, j));
               }
               num++;
            }
         }

         System.out.println ("number of source data: " + A.numBlockRows ());
         System.out.println ("number of target data: " + tgt.rowSize ());
         
         b.setSize (tgt.colSize () * tgt.rowSize ());
         int idx = 0;
         for (int i = 0; i < tgt.rowSize (); i++) {
            for (int j = 0; j < tgt.colSize (); j++) {
               b.set (idx++, tgt.get (i, j));
            }
         }

         super.applyMapWeights (A, b);
      }

      catch (Exception e) {
         e.printStackTrace ();
         return false;
      }

      return true;
   }


   boolean [] myIntMarks;
   /**
    * assemble data matrix according to the working sub-domain
    * @param domain
    * @param data
    * @return
    */
   private SparseBlockMatrix getSrcPntsSubdomainBasis () {

      if (myIntSrc.rowSize () != mySrcPnts.size ()) {
         throw new ImproperSizeException ("Incomaptible matrix size");
      }
      if (myIntSrc.colSize () != 3) {
         throw new ImproperSizeException ("Incomaptible matrix size");
      }

      myIntMarks = new boolean [myIntSrc.rowSize ()];
      
      SparseBlockMatrix Basis = new SparseBlockMatrix ();

      for (int i = 0; i < mySrcPnts.size (); i++) {
         if (mySrcPnts.get (i).isInsideSubdomain ()) {
            
           addBlockMatrixByRow (Basis, 
              mySrcPnts.get (i).createSubdomainBasisMatrix ());
           myIntMarks [i] = true;
           
         }
      }

      return Basis;
   }
   

}
