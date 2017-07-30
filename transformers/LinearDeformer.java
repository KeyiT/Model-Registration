package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.Set;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.TransformerSlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.LinearOptimizerSink;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.Vector;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public abstract class LinearDeformer extends RegistrationDeformer
implements LinearOptimizerSink{
   
   protected VectorNd myPosition;
   protected VectorNd myRestPosition;
   private int mySize = 0;

   public LinearDeformer () {
      super ();
      isFast = true;
      myPosition = new VectorNd ();
      myRestPosition = new VectorNd ();
   }

   @Override
   public void getDeformation (Vector3d p, Matrix3d F, Vector3d r) {
      throw new UnsupportedOperationException("");
   }

   @Override
   public void resetAction () {
      if (myPosition.size () != mySize 
      || myRestPosition.size () != mySize) {
         throw new ImproperSizeException ("");
      }
      myPosition.set (myRestPosition);
   }

   @Override
   public void fastTransformer (Set<SlaveInfo> slaves) {
   }

   @Override
   public void takeOutput (Vector output) {
      setPositions (output);
   }

   @Override
   public TransformerSlaveInfo createSubSlaveInfo (
      TransformableGeometry slave, SlaveInfo info) {
      return null;
   }

   public int getSize () {
      return mySize;
   }
   
   public void setSize (int size) {
      mySize = size;
      myPosition.setSize (mySize);
      myRestPosition.setSize (mySize);
   }
   
   public abstract Matrix createStiffnessMatrix ();
   
   public abstract Vector creatTargetForce ();
   
   public VectorNd getPositions () {
      return myPosition.clone ();
   }
   
   public void setPositions (Vector pos) {
      if (mySize != pos.size ()) {
         throw new ImproperSizeException ("output size: " 
      + pos.size () + "\n my size: " + mySize);
      }
      myPosition.set (pos);
   }
   
   public void advanceRestPositions () {
      myRestPosition.set (myPosition);
   }
}
