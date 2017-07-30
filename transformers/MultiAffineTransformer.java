package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.Set;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.TransformerSlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.AffineSink;
import maspack.geometry.GeometryTransformer;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Matrix3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

public class MultiAffineTransformer extends RegistrationTransformer
implements AffineSink{

   public MultiAffineTransformer () {
      enableFastTranform (false);
   }

   @Override
   public void takeOutput (AffineTransform3dBase output) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public TransformerSlaveInfo createSubSlaveInfo (
      TransformableGeometry slave, SlaveInfo info) {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void applyAction () {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void resetAction () {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void fastTransformer (Set<SlaveInfo> slaves) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean isRigid () {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public boolean isAffine () {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public boolean isInvertible () {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public GeometryTransformer getInverse () {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void computeTransformPnt (Point3d pr, Point3d p1) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeTransformVec (Vector3d vr, Vector3d v1, Vector3d r) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeTransform (RigidTransform3d TR, RigidTransform3d T1) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeTransform (AffineTransform3d XR, AffineTransform3d X1) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeTransform (
      RotationMatrix3d RR, RotationMatrix3d R1, Vector3d r) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeTransform (Matrix3d MR, Matrix3d M1, Vector3d r) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void computeTransform (Plane pr, Plane p1, Vector3d r) {
      // TODO Auto-generated method stub
      
   }

}
