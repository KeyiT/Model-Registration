package artisynth.models.swallowingRegistrationTool.utilities;

import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.workspace.RootModel;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;
import quickhull3d.QuickHull3D;

public class MeshConvexHull {
   
   PolygonalMesh myConvexHull;
   BVFeatureQuery myQuery = new BVFeatureQuery();
   
   public MeshConvexHull () {
   }
   
   public MeshConvexHull (MeshBase mesh) {
      build (mesh);
   }
   
   public MeshConvexHull (Point3d [] pnts) {
      build (pnts);
   }
   
   /**
    * generate convex hull for point cloud
    * @param pnts Point cloud
    * @return Convex hull represented as 
    * a polygonal mesh
    */
   public static PolygonalMesh generateConvexHull (
      Point3d [] pnts) {
      
      PolygonalMesh mesh = new PolygonalMesh ();
      
      quickhull3d.Point3d [] pntAgents = new quickhull3d.Point3d [pnts.length];
      for (int i = 0; i < pnts.length; i++) {
         pntAgents[i] = new quickhull3d.Point3d (
            pnts[i].x, pnts[i].y, pnts[i].z);
      }
      QuickHull3D hullGen = new QuickHull3D (pntAgents);
      // face and vertex
      int [][] faceInd = hullGen.getFaces ();
      quickhull3d.Point3d [] hullPnts = hullGen.getVertices ();
      // build convex hull
      for (quickhull3d.Point3d pnt : hullPnts) {
         Vertex3d vtx = new Vertex3d (
            new Point3d (pnt.x, pnt.y, pnt.z));
         mesh.addVertex (vtx);
      }
      for (int [] faceVI : faceInd) {
         mesh.addFace (faceVI);
      }
      if (! mesh.isTriangular ()) {
         mesh.triangulate ();
      }
      
      return mesh;
   }
   
   /**
    * generate convex hull for point cloud
    * @param pnts Point cloud
    * @return Convex hull represented as 
    * a polygonal mesh
    */
   public static PolygonalMesh generateConvexHull (
      quickhull3d.Point3d [] pnts) {
      
      PolygonalMesh mesh = new PolygonalMesh ();
      
      QuickHull3D hullGen = new QuickHull3D (pnts);
      // face and vertex
      int [][] faceInd = hullGen.getFaces ();
      quickhull3d.Point3d [] hullPnts = hullGen.getVertices ();
      // build convex hull
      for (quickhull3d.Point3d pnt : hullPnts) {
         Vertex3d vtx = new Vertex3d (
            new Point3d (pnt.x, pnt.y, pnt.z));
         mesh.addVertex (vtx);
      }
      for (int [] faceVI : faceInd) {
         mesh.addFace (faceVI);
      }
      if (! mesh.isTriangular ()) {
         mesh.triangulate ();
      }
      
      return mesh;
   }
   
   /**
    * generate convex hull for mesh
    * @param mesh 
    * @return Convex hull represented as 
    * a polygonal mesh
    */
   public static PolygonalMesh generateConvexHull (MeshBase mesh) {
      quickhull3d.Point3d [] pnts = 
      new quickhull3d.Point3d [mesh.getVertices ().size ()];
      int idx = 0;
      for (Vertex3d vtx : mesh.getVertices ()) {
         Point3d pnt = vtx.getWorldPoint ();
         quickhull3d.Point3d hullPnt = 
         new quickhull3d.Point3d (pnt.x, pnt.y, pnt.z);
         pnts[idx++] = hullPnt;
      }
      
      PolygonalMesh CHMesh = generateConvexHull(pnts);
      return CHMesh;
   }
   
   /**
    * generate convex hull for mesh
    * @param mesh 
    * 
    */
   public void build (MeshBase mesh) {
      myConvexHull = generateConvexHull (mesh);
   }
   
   /**
    * generate convex hull for point clouds
    * @param mesh 
    * 
    */
   public void build (Point3d[] pnts) {
      myConvexHull = generateConvexHull (pnts);
   }
   /**
    * compute distance between a point and it's projected position 
    * on convex hull.
    * @param pnt 
    * @return distance; 
    * if <tt>pnt</tt> is inside convex hull, the
    * distance would be negative otherwise it's positive;
    */
   public double computeDistanceToConvexHull (Point3d pnt) {
      if (myConvexHull == null) {
         throw new ImproperStateException ("Convex hull not initialized!");
      }
      
      Point3d nearest = new Point3d ();
      myQuery.nearestFaceToPoint (nearest, null, myConvexHull, pnt);
      double dis =  pnt.distance (nearest);
      if (BVFeatureQuery.isInsideOrientedMesh (myConvexHull, pnt)) {
         dis *= -1.0;
      }
      return dis;
   }
   
   public boolean isInsideConvexHull (Point3d pnt) {
      if (myConvexHull == null) {
         throw new ImproperStateException ("Convex hull not initialized!");
      }
      return BVFeatureQuery.isInsideOrientedMesh (myConvexHull, pnt);
   }
   
   public PolygonalMesh getConvexHullMesh () {
      return myConvexHull;
   }
   
   public void renderConvexHull (RootModel root) {
      if (myConvexHull == null) {
         throw new ImproperStateException ("Convex hull not initialized!");
      }
      FixedMeshBody agent = new FixedMeshBody ();
      agent.setName ("ConvexHullRenderAgent");
      agent.setMesh (myConvexHull);
      root.addRenderable (myConvexHull);
   }

}
