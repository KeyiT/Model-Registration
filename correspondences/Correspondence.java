package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.swallowingRegistrationTool.infoUtilities.ObjInfo;

public abstract class Correspondence extends ObjInfo{

   protected double myErr = 0;
   private boolean muted = false;

   /**
    * compute match error and save it;
    * @return mapping error 
    */
   public abstract double computeMatchError();

   /**
    * 
    * @return match error saved by {@link #computeMatchError}
    */
   public double getMatchError() {
      return myErr;
   }

   /**
    * need compute match error first!
    */
   public void printMatchError() {
      System.out.println ("Mapping Error: " + myErr);
   }
   
   public void mute (boolean enable) {
      muted = enable;
   }
   
   public boolean isMuted () {
      return muted;
   }

}
