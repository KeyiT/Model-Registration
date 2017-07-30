package artisynth.models.swallowingRegistrationTool.correspondences;

import maspack.matrix.VectorNd;

public interface InjectiveWeightsController {
   public VectorNd makeWeights (InjectiveMap feature);
}
