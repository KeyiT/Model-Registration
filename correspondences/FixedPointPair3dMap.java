package artisynth.models.swallowingRegistrationTool.correspondences;

import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class FixedPointPair3dMap<C extends VertexToPointCorrespondence> 
extends TrivariateInjectiveMap<C> {

   public FixedPointPair3dMap () {
      super ();
      isAllocatable = true;
   }

   @Override
   public boolean assignPartners (Point3d2Point3d correspondence) {
      VertexToPointCorrespondence feature = 
      (VertexToPointCorrespondence) correspondence;
      
      feature.setSelf (feature.getVertex ().getWorldPoint ());
      if (feature.getFeature () instanceof Vertex3d) {
         Point3d pnt = new Point3d ();
         Vertex3d vtx = (Vertex3d)feature.getFeature ();
         pnt.set (vtx.getWorldPoint ());
         feature.setPartner (pnt);
      }
      else {
         System.err.println ("Error: Incompatible partner feature!");
         return false;
      }
      
      return true;
   }

   @Override
   public Class<?> getAllocatableInfoClass () {
      return VertexToPointCorrespondence.class;
   }
}
