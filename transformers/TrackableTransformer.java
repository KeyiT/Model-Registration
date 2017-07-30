package artisynth.models.swallowingRegistrationTool.transformers;

import maspack.geometry.MeshBase;

public interface TrackableTransformer {
   public void startTracking ();
   public void startNewTracking ();
   public void endTracking ();
   public void makeAccumulatedTransform (MeshBase mesh);
}
