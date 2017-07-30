package artisynth.models.swallowingRegistrationTool.correspondences;

import java.util.ArrayList;

import artisynth.models.swallowingRegistrationTool.infoUtilities.CloudInfo;

public class CloudMap<T extends Correspondence> extends CloudInfo<T>{
   
   double myMapErr = 0;
   double myMaxErr = 0;

   @Override
   public boolean update () {
      if (super.update ()) {
         computeMappingError();
         return true;
      }
      return false;
   }

   /**
    * compute mapping error and save it;
    * @return mapping error 
    */
   public double computeMappingError () {
      double err = 0;
      ArrayList <T> infos = new ArrayList <T> ();
      getInfos (infos);
      int num = 0;
      for (T c : infos) {
         if (c.isMuted ()) {
            continue;
         }
         err += c.computeMatchError ();
         num++;
      }
      myMapErr = err/num;
      return myMapErr;
   }
   
   /**
    * compute maximum mapping error and save it;
    * @return maximum mapping error 
    */
   public double computeMaxMappingError () {
      double err = 0;
      ArrayList <T> infos = new ArrayList <T> ();
      getInfos (infos);
      int num = 0;
      for (T c : infos) {
         if (c.isMuted ()) {
            continue;
         }
         double tmp =  c.computeMatchError ();
         if (tmp > err) {
            err = tmp;
         }
      }
      myMaxErr = err;
      return myMaxErr;
   }
   
   /**
    * need compute match error first!
    */
   public double getMappingError () {
      return myMapErr;
   }
   
   /**
    * need compute maximum match error first!
    */
   public double getMaxMappingError () {
      return myMaxErr;
   }

}
