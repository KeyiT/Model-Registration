package artisynth.models.swallowingRegistrationTool.transformers;

import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

/**
 * point in nature coordinate;
 * undeformed position information saved in this class;
 * @author KeyiTang
 *
 */
public class FFDPoint3d extends Point3d {
   public BezierFFD3d myFFD;
   // undeformed position in world coordinate
   public Point3d myX;
   
   int myIndex = -1;

   public FFDPoint3d () {
      super ();
   }

   /**
    * 
    * @param vec natural coordinate
    */
   public FFDPoint3d (Vector3d vec) {
      super(vec);
   }

   public BezierFFD3d getFFD() {
      return myFFD;
   }

   public void setFFD(BezierFFD3d ffd) {
      myFFD = ffd;
   }
   
   public VectorNd computeBernsteinBasis () {
      return myFFD.myBPoly.makeBlendingFunctions (this);
   }
   
   public Point3d computeFFD () {
      Point3d Xffd = new Point3d (
         myFFD.myBPoly.getTenProBezCur(new VectorNd(this)));
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
   public void recomputeNaturalCoor () {
      if (myFFD  == null) {
         throw new ImproperStateException ("FFD not initialized");
      }
      if (myX == null) {
         throw new ImproperStateException ("Undeformed Position not initialized");
      }
      this.set (myFFD.car2stu (myX));
   }
}
