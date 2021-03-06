package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Set;

import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.swallowingRegistrationTool.infoUtilities.*;
import artisynth.models.swallowingRegistrationTool.optimizers.AffineSink;
import maspack.geometry.AffineTransformer;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.AffineTransform3dBase;
import maspack.matrix.Matrix3d;
import maspack.matrix.Plane;
import maspack.matrix.Point3d;
import maspack.matrix.PolarDecomposition3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

public class AffineModelTransformer extends RegistrationTransformer
implements TrackableTransformer, AffineSink{

   private AffineTransform3d myX;
   private PolarDecomposition3d myPolarD;
   private AffineTransform3d myInvX;
   private boolean myInvertibleP;


   protected ArrayList <MeshBase> myOldMeshes = new ArrayList<MeshBase> ();

   private boolean isTracking = false;
   private AffineTransform3d myAccX = 
   new AffineTransform3d ();


   public AffineModelTransformer () {
      enableFastTranform (false);
   }

   public AffineModelTransformer (AffineTransform3d X) {
      enableFastTranform (false);
      setTransform(X);
   }

   public void setTransform (AffineTransform3dBase X) {
      myX = new AffineTransform3d (X);
      myPolarD = new PolarDecomposition3d();
      myPolarD.factorLeft (X.getMatrix ());
      if (isTracking) {
         myAccX.mul (myX, myAccX);
      }
   }


   @Override
   public void applyAction () {
      if (myMeshes.size () == 0) {
         throw new NullPointerException ("No master");
      }
      for (MeshBase mesh : myMeshes) {
         mesh.transform (myX);
      }
   }

   @Override
   // TODO: maybe change action mechanism later
   // use accumulated transformation method
   public void resetAction () {
      if (myMeshes.size () == 0) {
         throw new NullPointerException ("No master");
      }

      for (MeshBase mesh : myMeshes) {
         int idx = 0;
         MeshBase oldMesh = myOldMeshes.get (myMeshes.indexOf (mesh));
         for (Vertex3d vtx : mesh.getVertices ()) {
            vtx.setPosition (oldMesh.getVertex (idx).pnt);
            idx++;
         }
         idx = 0;
         for (Vector3d normal : mesh.getNormals ()) {
            normal.set (oldMesh.getNormal (idx++));
         }
      }

   }

   @Override
   public void startTracking () {
      isTracking = true;
   }

   public void startNewTracking () {
      isTracking = true;
      myAccX.setIdentity ();
   }

   @Override
   public void endTracking () {
      isTracking = false;
   }

   @Override
   public void fastTransformer (Set<SlaveInfo> slaves) {
   }

   //TODO
   public void makeAccumulatedTransform (TransformableGeometry geometry) {
      geometry.transformGeometry (myAccX);
   }


   public void makeAccumulatedTransform (MeshBase mesh) {
      mesh.transform (myAccX);
   }

   public void makeAccumulatedTransform (Point3d pnt) {
      pnt.transform (myAccX);
   }

   public AffineTransform3d getAccumulatedTransform () {
      return new AffineTransform3d (myAccX);
   }

   public void addMasterMesh (MeshBase master) {
      super.addMasterMesh (master);
      myOldMeshes.add(master.copy ());
   }

   @Override
   public void takeOutput (AffineTransform3dBase output) {
      setTransform (output);
   }

   @Override
   public TransformerSlaveInfo<AffineModelTransformer> createSubSlaveInfo (
      TransformableGeometry slave, SlaveInfo info) {

      TransformerSlaveInfo<AffineModelTransformer> tInfo = 
      new TransformerSlaveInfo<AffineModelTransformer> ();
      tInfo.setSlaveInfo (info);
      tInfo.setTransformer (this);
      return tInfo;
   }

   //----------------------------------------------------------------------//
   // Following code is copied from maspack.geometry.AffineTransformer
   //----------------------------------------------------------------------//

   /**
    * Returns <code>false</code>, since this transformer does not implement a
    * linear rigid transform.
    */
   public boolean isRigid() {
      return false;
   }

   /**
    * Returns <code>true</code>, since this transformer does implement a
    * linear affine transform.
    */
   public boolean isAffine() {
      return true;
   }

   /**
    * Returns <code>true</code>, since this transformer is invertible.
    */
   public boolean isInvertible() {
      return true;
   }

   private void updateInverse() {
      if (myInvX == null) {
         myInvX = new AffineTransform3d (myX);
         myInvertibleP = myInvX.invert();
      }
   }

   /**
    * Returns a transformer that implements the inverse operation of this
    * transformer.
    * 
    * @return inverse transformer
    */
   public AffineTransformer getInverse() {
      updateInverse();
      return new AffineTransformer (myInvX);
   }

   /**
    * Transforms a point <code>p1</code> and returns the result in
    * <code>pr</code>. The transform is computed according to
    * <pre>
    * pr = F p1 + pf
    * </pre>
    * This method provides the low level implementation for point
    * transformations and does not do any saving or restoring of data.
    * 
    * @param pr transformed point
    * @param p1 point to be transformed
    */
   public void computeTransformPnt (Point3d pr, Point3d p1) {
      pr.transform (myX, p1);
   }

   /**
    * Transforms a vector <code>v1</code>, and returns the result in
    * <code>vr</code>. 
    * The transform is computed according to
    * <pre>
    * vr = F v1
    * </pre>
    * The reference position is ignored since affine transforms are position
    * invariant.
    *
    * This method provides the low level implementation for vector
    * transformations and does not do any saving or restoring of data.
    *
    * @param vr transformed vector
    * @param v1 vector to be transformed
    * @param r reference position of the vector (ignored)
    */
   public void computeTransformVec (Vector3d vr, Vector3d v1, Vector3d r) {
      vr.transform (myX, v1);
   }

   /**
    * Transforms a rigid transform <code>T1</code> and returns the result in
    * <code>TR</code>. If
    * <pre>
    *      [  R1   p1 ]
    * T1 = [          ]
    *      [  0    1  ]
    * </pre>
    * the transform is computed according to
    * <pre>
    *      [  RF R1   F p1 + pf ]
    * TR = [                    ]
    *      [    0          1    ]
    * </pre>
    * where PF RF = F is the left polar decomposition of F.
    * 
    * This method provides the low level implementation for the transformation
    * of rigid transforms and does not do any saving or restoring of data.
    *
    * @param TR transformed transform
    * @param T1 transform to be transformed
    */
   public void computeTransform (RigidTransform3d TR, RigidTransform3d T1) {
      TR.set (T1);
      TR.mulAffineLeft (myX, myPolarD.getR());
   }

   /**
    * Transforms an affine transform <code>X1</code> and returns the result in
    * <code>XR</code>. If
    * <pre>
    *      [  A1   p1 ]
    * X1 = [          ]
    *      [  0    1  ]
    * </pre>
    * the transform is computed according to
    * <pre>
    *      [  F A1   F p1 + pf ]
    * XR = [                   ]
    *      [   0         1     ]
    * </pre>
    *
    * This method provides the low level implementation for the transformation
    * of affine transforms and does not do any saving or restoring of data.
    * 
    * @param XR transformed transform
    * @param X1 transform to be transformed
    */
   public void computeTransform (AffineTransform3d XR, AffineTransform3d X1) {
      XR.mul (myX, X1);
   }

   /**
    * Transforms a rotation matrix <code>R1</code>, located at reference
    * position <code>ref</code>, and returns the result in <code>RR</code>.
    * The transform is computed according to
    * <pre>
    * RR = RF R1
    * </pre>
    * where PF RF = F is the left polar decomposition of F.
    * The reference position is ignored since affine transforms are position
    * invariant.
    * 
    * This method provides the low level implementation for the transformation
    * of rotation matrices and does not do any saving or restoring of data.
    *
    * @param RR transformed rotation
    * @param R1 rotation to be transformed
    * @param r reference position of the rotation (ignored)
    */
   public void computeTransform (
      RotationMatrix3d RR, RotationMatrix3d R1, Vector3d r) {

      RR.mul (myPolarD.getR(), R1);
   }

   /**
    * Transforms a general 3 X 3 matrix <code>M1</code>, located at reference
    * position <code>ref</code>, and returns the result in <code>MR</code>.
    * The transform is computed according to
    * <pre>
    * MR = F M1
    * </pre>
    * The reference position is ignored since affine transforms are position
    * invariant.
    * 
    * This method provides the low level implementation for the transformation
    * of 3 X 3 matrices and does not do any saving or restoring of data.
    *
    * @param MR transformed matrix
    * @param M1 matrix to be transformed
    * @param r reference position of the matrix (ignored)
    */
   public void computeTransform (Matrix3d MR, Matrix3d M1, Vector3d r) {

      MR.mul (myX.A, M1);
   }

   /**
    * Transforms a plane <code>p1</code>, located at reference position
    * <code>ref</code>, and returns the result in <code>pr</code>.
    * Assume that <code>p1</code> is defined by a normal <code>n1</code>
    * and offset <code>o1</code> such that all planar points <code>x</code>
    * satisfy
    * <pre>
    * n1^T x = o1
    * </pre>
    * Then the transformed normal <code>nr</code> and offset <code>or</code>
    * are computed according to
    * <pre>
    * nr = inv(F)^T n1
    * or = o1 + nr^T pf
    * mag = ||nr||
    * nr = nr/mag, or = or/mag
    * </pre>
    * The reference position is ignored since affine transforms are position
    * invariant.
    *
    * This method provides the low level implementation for the transformation
    * of planes and does not do any saving or restoring of data.
    *
    * @param pr transformed plane
    * @param p1 plane to be transformed
    * @param r reference position of the plane (ignored)
    */
   public void computeTransform (Plane pr, Plane p1, Vector3d r) {
      pr.transform (myX, p1);
   }

}
