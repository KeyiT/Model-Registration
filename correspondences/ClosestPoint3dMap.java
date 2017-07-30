package artisynth.models.swallowingRegistrationTool.correspondences;

import maspack.matrix.Point3d;

public class ClosestPoint3dMap<C extends VertexToMeshCorrespondence> extends 
TrivariateInjectiveMap<C> {

   protected boolean srcToTgtMapping = true;
   protected boolean tgtToSrcMapping = true;

   public ClosestPoint3dMap () {
      super ();
      isAllocatable = true;
   }

   @Override
   public boolean assignPartners (Point3d2Point3d correspondence) {
      if (! (correspondence instanceof VertexToMeshCorrespondence)) {
         throw new IllegalArgumentException (
            "Incompatible correspondence");
      }

      VertexToMeshCorrespondence feature = 
      (VertexToMeshCorrespondence) correspondence;

      if (isMuted(feature)) {
         return false;
      }

      if (feature.isLocked ()) {
         return true;
      }

      Point3d vtx = new Point3d (feature.getVertex ().
         getWorldPoint ());
      Point3d near = new Point3d (feature.
         getNearestPoint ());

      if (feature.getDirection ()) {
         feature.setSelf (vtx);
         feature.setPartner (near);
      }
      else {
         feature.setSelf (near);
         feature.setPartner (vtx);
      }
      return true;
   }

   @Override
   public Class<?> getAllocatableInfoClass () {
      return VertexToMeshCorrespondence.class;
   }

   public void enableSourceToTargetMapping (boolean enable) {
      srcToTgtMapping = enable;
   }

   public void enableTargetToSourceMapping (boolean enable) {
      tgtToSrcMapping = enable;
   }

   public boolean isSourceToTargetEnabled () {
      return srcToTgtMapping;
   }

   public boolean isTargetToSourceEnabled () {
      return tgtToSrcMapping;
   }

   protected boolean isMuted (
      VertexToMeshCorrespondence feature) {

      if (srcToTgtMapping && feature.getDirection ()) {
         feature.mute (false);
         return false;
      }
      if (tgtToSrcMapping && (!feature.getDirection ())) {
         feature.mute (false);
         return false;
      }
      
      feature.mute (true);
      return true;
   }

}
