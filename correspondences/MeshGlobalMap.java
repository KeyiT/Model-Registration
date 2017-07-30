package artisynth.models.swallowingRegistrationTool.correspondences;

public abstract class MeshGlobalMap<C extends MeshToMeshCorrespondence> 
extends CloudMap<C> {
   
   @Override
   public abstract double computeMappingError ();
   
}
