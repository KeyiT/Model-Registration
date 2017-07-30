package artisynth.models.swallowingRegistrationTool.transformers;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;

public class NFFDPoint3d extends Point3d{

   public NFFD3d myFFD;
   // undeformed position in world coordinate
   public Point3d myX;
   SparseBlockMatrix Basis;
   
   int myIndex = -1;

   public NFFDPoint3d () {
      super ();
   }

   /**
    * 
    * @param vec natural coordinate
    */
   public NFFDPoint3d (Vector3d vec) {
      super(vec);
   }

   public NFFD3d getFFD() {
      return myFFD;
   }

   public void setFFD(NFFD3d ffd) {
      myFFD = ffd;
   }
   
   public SparseBlockMatrix createBasisMatrix () {
      if (Basis != null) {
         return Basis;
      }
      return myFFD.createBasisMatrix (this);
   }
   
   public SparseBlockMatrix createSubdomainBasisMatrix () {
      return myFFD.createSubBasisMatrix (this);
   }
   
   public Point3d computeFFD () {
      if (myFFD == null) {
         throw new ImproperStateException ("FFD not initialized!");
      }
      Point3d Xffd = new Point3d ();
      
      if (Basis == null) {
         myFFD.eval (Xffd, this);
      }
      else {
         Vector4d [] ps = myFFD.getControlPoints ();
         VectorNd P = new VectorNd (ps.length * 3);
         int idx = 0;
         for (int i = 0; i < ps.length; i++) {
            Vector4d p4 = ps[i];
            for (int j = 0; j < 3; j++) {
               P.set (idx, p4.get (j));
               idx++;
            }
         }
         
         VectorNd vr = new VectorNd ();
         Basis.mul (vr, P);
         //TODO: test
         //Xffd.set (vr);
         myFFD.eval (Xffd, this);
      }
      
      return Xffd;
   }
   
   public void setUndeformedPosition (Vector3d X) {
      myX = new Point3d(X);
   }
   
   public Point3d getUndeformedPosition () {
      return new Point3d(myX);
   }
   
   /**
    * used for arranging FFD point in order.
    * default value is -1
    * @param idx
    */
   public void setIndex (int idx) {
      myIndex = idx;
   }
   
   /**
    * used for arranging FFD point in order.
    * default value is -1
    * @return
    */
   public int getIndex () {
      return myIndex;
   }
   
   /**
    * compute natural coordinate and use it to set
    * this point. If FFD or undeformed position was
    * not specified throw exception.
    */
   public void evalUVT () throws FFDPointNotFoundException{
      if (myFFD  == null) {
         throw new ImproperStateException (
            "FFD not initialized");
      }
      if (myX == null) {
         throw new ImproperStateException (
            "Undeformed Position not initialized");
      }
      
      Point3d uvt = myFFD.findPoint (myX);
      if (uvt == null) {
         throw new FFDPointNotFoundException (
            "Failed to evalue uvt");
      }
      set (uvt);
      
      Basis = myFFD.createBasisMatrix (this);
   }
   
   public boolean isInsideSubdomain () {
      if (myFFD  == null) {
         throw new ImproperStateException (
            "FFD not initialized");
      }
      if (myX == null) {
         throw new ImproperStateException (
            "Undeformed Position not initialized");
      }
      
      return myFFD.isInsideSubdomain (this);
   }

}
