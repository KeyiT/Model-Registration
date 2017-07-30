package artisynth.models.swallowingRegistrationTool.optimizers;

import maspack.matrix.AffineTransform3dBase;

public interface AffineSink extends OptimizerSink{
   public void takeOutput (AffineTransform3dBase output);
}
