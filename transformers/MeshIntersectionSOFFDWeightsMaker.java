package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import artisynth.core.mechmodels.Point;
import maspack.collision.IntersectionContour;
import maspack.collision.IntersectionPoint;
import maspack.collision.SurfaceMeshIntersector;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public class MeshIntersectionSOFFDWeightsMaker extends SOFFDWeightsMaker{
   
   List<PolygonalMesh> myCutMeshes = new ArrayList<PolygonalMesh> ();;
   
   public MeshIntersectionSOFFDWeightsMaker (
      List<PolygonalMesh> cutMeshes) {
      myCutMeshes.addAll (cutMeshes);
   }

   @Override
   public LinkedHashMap<List<Face>,double[]> makeReferenceAndWeights (
      List<Point> points, PolygonalMesh mesh) {
      
      SurfaceMeshIntersector smi = new SurfaceMeshIntersector ();
      BVFeatureQuery bv = new BVFeatureQuery ();
      LinkedHashMap<List<Face>,double[]> map = new LinkedHashMap<List<Face>,double[]> ();
      
      if (points.size () != myCutMeshes.size ()) {
         throw new ImproperSizeException ("");
      }
      
      for (int i = 0; i < points.size (); i++) {
         Point pnt = points.get (i);
         PolygonalMesh cut = myCutMeshes.get (i);
         List<Face> faces = new ArrayList<Face>();
         
         
         ArrayList<IntersectionContour> icts = 
         smi.findContours (cut, mesh);
         for (IntersectionContour ict : icts) {
            for (IntersectionPoint ipnt : ict) {
               if (!faces.contains (ipnt.face)) {
                  if (mesh.getFaces ().contains (ipnt.face)) {
                     faces.add (ipnt.face);
                  }
               }
            }
         }
         
         double [] ws = new double [faces.size ()];
         double [] diss = new double [faces.size ()];
         double dmin = Double.MAX_VALUE;
         int idx = 0;
         for (Face face : faces) {
            Point3d ppnt = new Point3d ();
            face.nearestPoint (ppnt, pnt.getPosition ());
            diss [idx] = ppnt.distance (pnt.getPosition ());
            if (dmin > diss[idx]) {
               dmin =  diss[idx];
            }
            idx++;
         }
         
         idx = 0;
         double sum = 0;
         for (double dis : diss) {
            ws[idx] = dmin / dis;
            sum += ws[idx++];
         }
         for (idx = 0; idx < ws.length; idx++) {
            ws[idx] = ws[idx] / sum;
         }
         
         map.put (faces, ws);
         // TODO: test
         System.out.println (new VectorNd (ws));
      }
      
      
      return map;
   }

}
