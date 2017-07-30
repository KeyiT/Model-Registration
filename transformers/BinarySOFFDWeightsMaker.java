package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import artisynth.core.mechmodels.Point;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;

public class BinarySOFFDWeightsMaker extends SOFFDWeightsMaker{
   
   LinkedHashMap<Point, Face> myMap = new LinkedHashMap<Point, Face> ();
   HashMap<Face, Face> cur2Udf = new HashMap<Face, Face> ();

   @Override
   public LinkedHashMap<List<Face>,double[]> makeReferenceAndWeights (
      List<Point> points, PolygonalMesh mesh) {
      BVFeatureQuery bv = new BVFeatureQuery ();
      LinkedHashMap<List<Face>,double[]> map = new LinkedHashMap<List<Face>,double[]> ();
      
      for (int i = 0; i < points.size (); i++) {
         Point pnt = points.get (i);
         List<Face> faces = new ArrayList<Face>(1);
         double [] ws = new double [1];
         
         Point3d near = new Point3d ();
         Face face = bv.nearestFaceToPoint (near, null, mesh, pnt.getPosition ());
         //pnt.setPosition (near);
         ws [0] = 1;
         faces.add (face);
         
         myMap.put (pnt, face);
         Face un = new Face (face.getIndex ());
         
         Vertex3d [] vtxs = new Vertex3d [face.numVertices ()];
         for (int j = 0; j < vtxs.length; j++) {
            Point3d unpos = new Point3d (face.getVertex (j).getWorldPoint ());
            vtxs [j] = new Vertex3d (unpos);
         }
         un.set (vtxs, vtxs.length, true);
         cur2Udf.put (face, un);
         map.put (faces, ws);
      }
      return map;
   }
   
   public void updatePoints () {
      for (Point pnt : myMap.keySet ()) {
         AffineTransform3d invD = new AffineTransform3d ();
         Face face1 = myMap.get (pnt);
         Face face0 = cur2Udf.get (face1);
         invD.mulInverseRight (computeFacePosInWorld (face1), 
            computeFacePosInWorld (face0));
         Point3d pos = new Point3d (pnt.getPosition ());
         pos.transform (invD);
         pnt.setPosition (pos);
         for (int j = 0; j < myMap.get (pnt).numVertices (); j++) {
            face0.getVertices () [j].setPosition (
               face1.getVertex (j).getPosition ());
         }
      }
   }
   
   private AffineTransform3d computeFacePosInWorld (Face face) {
      if (face.isTriangle ()) {
         Vertex3d [] vtxs = face.getTriVertices ();
         Vector3d v1 = new Vector3d ();
         Vector3d v2 = new Vector3d ();
         Vector3d v3 = new Vector3d ();
         Point3d p1 = vtxs[0].getWorldPoint ();
         Point3d p2 = vtxs[1].getWorldPoint ();
         Point3d p3 = vtxs[2].getWorldPoint ();
         v1.sub (p2, p1);
         v2.sub (p3, p1);
         v3.cross (v1, v2);
         v3.normalize ();
         AffineTransform3d X = new AffineTransform3d ();
         X.A.setColumns (v1, v2, v3);
         X.p.set (p1);
         return X;
      }
      else {
         return null;
      }
   }

}
