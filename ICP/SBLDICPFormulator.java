package artisynth.models.swallowingRegistrationTool.ICP;


import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.correspondences.TrivariateInjectiveMap;
import artisynth.models.swallowingRegistrationTool.transformers.BlendLinear3dDeformer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.SubdividableBL3dDeformer;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix;
import maspack.matrix.MatrixNd;
import maspack.matrix.VectorNd;

public class SBLDICPFormulator extends BLDICPFormulator{

   private SubdividableBL3dDeformer myDeformer;

   protected int myWorkDomain = 0;
   private MatrixNd myIntSrc;
   private MatrixNd myIntTgt;

   private int [][] myIntSubdomains;
   protected boolean dataValidP = false;


   @Override
   public void setTransformer (RegistrationTransformer tf) {

      super.setTransformer (tf);

      if (tf instanceof SubdividableBL3dDeformer) {
         myDeformer = (SubdividableBL3dDeformer)tf;
         return;
      }

      System.out.println ("Not dividable!");
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

   public void validateSubdomainData (boolean valid) {
      dataValidP = valid;
   }

   public boolean isSubdomainDataValid () {
      return dataValidP;
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

            myIntSrc = myMap.makeSourceDataMatrix ();
            myIntTgt = myMap.makeTargetedChangeMatrix ();

            myIntSubdomains = myDeformer.findSubdomains (myIntSrc);
            if (myIntSubdomains == null) {
               System.err.println (
                  "Failed to find sub-domains for source points!");
               return false;
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
         MatrixNd src = new MatrixNd ();
         MatrixNd tgt = new MatrixNd ();
         boolean [] marks = getSubdomainData (src, myWorkDomain);

         if (src.rowSize () == 0) {
            A.set (new SparseBlockMatrix ());
            b.setSize (0);
            return true;
         }

         int num = 0;
         for (int i = 0; i < myIntTgt.rowSize (); i++) {
            if (marks[i]) {
               tgt.setSize (num+1, 3);
               for (int j = 0; j < 3; j++) {
                  tgt.set (num, j, myIntTgt.get (i, j));
               }
               num++;
            }
         }

         System.out.println ("number of source data: " + src.rowSize ());
         System.out.println ("number of target data: " + tgt.rowSize ());

         Matrix Basis = myDeformer.makeSubdomainBasis  (src);

         if (Basis == null) {
            return false;
         }

         A.set (Basis);

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


   /**
    * assemble data matrix according to the working sub-domain
    * @param domain
    * @param data
    * @return
    */
   private boolean [] getSubdomainData (MatrixNd D, int domain) {
      D.setSize (0, 3);

      if (myIntSrc.rowSize () != myIntSubdomains.length) {
         throw new ImproperSizeException ("Incomaptible matrix size");
      }
      if (myIntSrc.colSize () != 3) {
         throw new ImproperSizeException ("Incomaptible matrix size");
      }

      boolean [] marks = new boolean [myIntSrc.rowSize ()];

      for (int i = 0; i < myIntSubdomains.length; i++) {
         for (int d : myIntSubdomains[i]) {
            if (d == domain) {
               D.setSize (D.rowSize () + 1, 3);
               for (int j = 0; j < 3; j++) {
                  D.set (D.rowSize ()-1, j, myIntSrc.get (i, j));
               }
               marks[i] = true;
               break;
            }
         }
      }

      return marks;
   }


}
