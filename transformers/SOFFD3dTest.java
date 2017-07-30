package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.models.frank2.FrankModel2;
import artisynth.models.frank3.FrankModel3;
import artisynth.models.subjectFrank.SubjectModel;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;

public class SOFFD3dTest extends BezerFFD3dTest{
   
   SOFFD3d mySOFFD;
   PointList <Point> pntList;
   
   public void build (String [] args) throws IOException {
      
      addMesh (SubjectModel.class, "subjects/template/geometry/airway/airwayMesh.ply");
      setRenderProperties(meshList, Color.CYAN);
      addRenderable (makeFMB (meshList.get (0)));
      meshList.get (0).setFixed (false);
      
      PolygonalMesh poly = (PolygonalMesh)meshList.get (0);
      RigidTransform3d Xmw = poly.computePrincipalAxes ();
      meshList.get (0).inverseTransform (Xmw);
      
      MeshBase mesh = generatePointCloud (0.2, 20);
      pntList = makePointList (mesh);
      addRenderable (pntList);
      
      
      myFFD = new BezierFFD3d(3, 3, 3, meshList.get (0));
      addModel(myFFD);
      myPanel = myFFD.createControlPanel ();
      mySOFFD = new SOFFD3d (poly);
      mySOFFD.addSlavePoints (pntList);
      mySOFFD.updateMaps ();
      mySOFFD.addSOFFDWidgetsToControlPanel (myPanel);
      addController (mySOFFD);
      addControlPanel(myPanel);
   }
   
   public static ArrayList <Point3d> makeMeshPointList (MeshBase mesh) {
      ArrayList <Point3d> list = new ArrayList <Point3d> ();
      for (Vertex3d vtx: mesh.getVertices ()) {
         Point3d pnt = new Point3d ();
         vtx.getWorldPoint (pnt);
         list.add (pnt);
      }
      return list;
   }
   
   public static PointList <Point> makePointList (MeshBase mesh) {
      PointList <Point> list = new PointList <Point> (Point.class);
      for (Vertex3d vtx: mesh.getVertices ()) {
         Point3d pnt = new Point3d ();
         vtx.getWorldPoint (pnt);
         Point pp = new Point (pnt);
         renderPoint (pp, Color.BLUE);
         list.add (pp);
      }
      return list;
   }
   
   public static void renderPoint (Point pnt, Color color) {
      RenderProps.setPointStyle (pnt, PointStyle.SPHERE);
      RenderProps.setPointRadius (pnt, 0.0002);
      RenderProps.setPointColor (pnt, color);
   }

}
