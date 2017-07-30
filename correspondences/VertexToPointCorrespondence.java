package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.swallowingRegistrationTool.infoUtilities.VertexAllocatable;
import maspack.geometry.Feature;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;

public class VertexToPointCorrespondence extends Correspondence
implements Point3d2Point3d, VertexAllocatable{
   
   protected Vertex3d myVtx;
   protected Feature myFeature;
   protected Point3d myPnt;
   protected Point3d myPartner;
   protected Point3d mySelf;
   protected MeshBase myMesh;
   protected MeshBase myTargetMesh;

   protected boolean locked = false;
   protected boolean direction = true;
   
   public VertexToPointCorrespondence () {
      super();
   }

   @Override
   public Point3d getPartner () {
      return myPartner;
   }

   @Override
   public void setPartner (Point3d partner) {
      if (locked) {
         System.out.println ("warning: partner locked");
         return;
      }
      myPartner = partner;
   }

   @Override
   public Point3d getSelf () {
      return mySelf;
   }

   @Override
   public void setSelf (Point3d self) {
      if (locked) {
         System.out.println ("warning: self locked");
         return;
      }
      mySelf = self;
   }

   @Override
   public void lockRelationship (boolean lock) {
      locked = lock;
   }

   @Override
   public boolean isLocked () {
      return locked;
   }

   @Override
   public void assignFeature (Vertex3d feature) {
      myVtx = feature;
      myMesh = myVtx.getMesh ();
   }

   @Override
   public void assignMesh (MeshBase mesh) {
      myTargetMesh = mesh;
   }

   @Override
   public void setDirection (boolean positive) {
      direction = positive;
   }
   
   public boolean getDirection () {
      return direction;
   }

   @Override
   public double computeMatchError () {
      if (mySelf != null && myPartner != null) {
         myErr = mySelf.distance (myPartner);
         return myErr;
      }
      else {
         myErr = Double.NaN;
         return myErr;
      }
   }
   
   public Feature getFeature () {
      return myFeature;
   }
   
   public Vertex3d getVertex () {
      return myVtx;
   }
   
   @Override
   public boolean update () {
      if (myVtx != null) {
         int idx = myVtx.getIndex ();
         myFeature = myTargetMesh.getVertex (idx);
         myPnt = new Point3d (myTargetMesh.
            getVertex (idx).getWorldPoint ());
      }
      
      return true;
   }

}
