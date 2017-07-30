package artisynth.models.swallowingRegistrationTool.infoUtilities;

import maspack.geometry.MeshBase;

public interface MeshFeatureAllocatable <T> {
   
   public void assignFeature (T feature);
   
   public void assignMesh (MeshBase mesh);
   
   public void setDirection (boolean positive);
}
