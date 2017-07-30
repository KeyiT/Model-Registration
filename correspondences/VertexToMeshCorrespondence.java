package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.swallowingRegistrationTool.infoUtilities.VertexAllocatable;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.Feature;
import maspack.geometry.MeshBase;
import maspack.geometry.OBBTree;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;

public class VertexToMeshCorrespondence extends Correspondence
implements Point3d2Point3d, VertexAllocatable{

   protected Vertex3d myVertex;
   protected MeshBase myMesh;
   protected MeshBase myTargetMesh;
   protected Face nearestFace;
   protected Vertex3d nearestVertex;
   protected Vector2d coor = new Vector2d ();
   protected Point3d nearestPnt = new Point3d();
   
   protected Point3d myPartner;
   protected Point3d mySelf;
   protected boolean locked = false;
   
   private boolean direction = true;
   
   public VertexToMeshCorrespondence() {
      super();
   }
   
   public VertexToMeshCorrespondence(Vertex3d vtx, MeshBase tgtMesh) {
      myTargetMesh = tgtMesh;
      myVertex = vtx;
      findClosestPoint();
      computeMatchError();
   }
   

   @Override
   public boolean update () {
      findClosestPoint();
      computeMatchError();
      return true;
   }

   @Override
   /**
    * distance between self vertex and 
    * nearest point on the target mesh
    */
   public double computeMatchError () {
      Point3d pnt3d = myVertex.getWorldPoint ();
      myErr = pnt3d.distance (nearestPnt);
      //System.out.println ("vertex: " + pnt3d);
      //System.out.println ("nearest: " + nearestPnt);
      //System.out.println (myErr);
      return myErr;
   }
   
   @Override
   public void assignFeature (Vertex3d vtx) {
      myVertex = vtx;
      myMesh = myVertex.getMesh ();
   }
   
   @Override
   public void assignMesh (MeshBase mesh) {
      myTargetMesh = mesh;
   }
   
   @Override
   public void setDirection (boolean positive) {
      direction = positive;
   }
   

   @Override
   public Point3d getPartner () {
      return myPartner;
   }


   @Override
   public void setPartner (Point3d partner) {
      if (! locked)  myPartner = partner;
      else System.out.println ("Partner locked!");
   }


   @Override
   public Point3d getSelf () {
      return mySelf;
   }


   @Override
   public void setSelf (Point3d self)  {
      if (!locked) mySelf = self;
      else System.out.println ("Self locked!");
   }


   @Override
   public void lockRelationship (boolean lock) {
     locked = lock;
   }
   
   @Override
   public boolean isLocked () {
      return locked;
   }


   
   public boolean getDirection () {
      return direction;
   }
   
   public Vertex3d getVertex () {
      return myVertex;
   }
   
   public MeshBase getTargetMesh() {
      return myTargetMesh;
   }
   
   public MeshBase getMesh () {
      return myMesh;
   }
   
   /**
    * 
    * @return mesh feature, 
    * nearest face or vertex
    */
   public Feature getNearestFeature() {
      if (nearestFace != null) {
         return nearestFace;
      }
      else {
         return nearestVertex;
      }
   }
   
   public Point3d getNearestPoint() {
      return nearestPnt; 
   }
   
   public Vector2d getBarycentric () {
      return coor;
   }
   
   
   /**
    * Given vertex world position p, find the closest point of
    * p on the target mesh saved in this feature;
    * If target mesh is a polygonal mesh, save the closest point 
    * position and the closest face; 
    * Otherwise save the closest point position and the closest 
    * vertex;
    */
   public void findClosestPoint () {
      findClosestPoint (myTargetMesh);
   }
   
   /**
    * Given vertex world position p, find the closest point of
    * p on the <tt>mesh</tt>;
    * If mesh is a polygonal mesh, save the closest point position
    * and the closest face; Otherwise save the closest point position
    * and the closest vertex;
    * @param mesh
    */
   public void findClosestPoint (MeshBase mesh) {
      findClosestPoint (mesh, null);
   }
   
   /**
    * Given vertex world position p, find the closest point of
    * X(p) on the target mesh saved in this feature, X() is 3D 
    * affineTransformation; If the target mesh is a polygonal 
    * mesh, save the closest point position and the closest face; 
    * Otherwise save the closest point position and the closest 
    * vertex;
    * @param X affine transformation, if null find the closest point
    * of p on <tt>mesh</tt>;
    */
   public void findClosestPoint (AffineTransform3d X) {
      findClosestPoint (myTargetMesh, X);
   }
   
   /**
    * Given vertex world position p, find the closest point of
    * X(p) on <tt>mesh</tt>, X() is 3D affineTransformation; 
    * If mesh is a polygonal mesh, save the closest point position
    * and the closest face; Otherwise save the closest point position
    * and the closest vertex;
    * @param mesh
    * @param X affine transformation, if null find the closest point
    * of p on <tt>mesh</tt>;
    */
   public void findClosestPoint (MeshBase mesh, AffineTransform3d X) {
      BVFeatureQuery query = new BVFeatureQuery();
      if (mesh instanceof PolygonalMesh) {
         PolygonalMesh poly = (PolygonalMesh)mesh;
         Point3d pnt = new Point3d (myVertex.getWorldPoint ());
         if (X != null) {
            pnt.transform (X);
         }
         nearestFace = query.nearestFaceToPoint (
            nearestPnt, coor, poly, pnt); 
         nearestVertex = null;
      }
      else {
         OBBTree tree = new OBBTree (mesh);
         nearestVertex = query.nearestVertexToPoint (tree, 
            myVertex.getWorldPoint ());
         nearestPnt.set (nearestVertex.getWorldPoint ());
         nearestFace = null;
      }
   }



}
