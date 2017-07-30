package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank3.FrankModel3;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import artisynth.models.swollowingFrank.MeshExperiment;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PointMesh;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

public class NFFDTest extends RootModel{

   ArrayList<MeshBase> meshList = new ArrayList<MeshBase> ();
   protected NFFDController myFFD;

   ControlPanel myPanel = new ControlPanel();

   protected RenderProps controlPointsRenderProps = new RenderProps ();


   public void build(String [] args) throws IOException {
      super.build (args);

      //addMesh (FrankModel3.class, "geometry/softPalate/palateSurface_rst.obj");
      addMesh (FrankModel3.class, "geometry/C0.obj");
      
      //Point3d cen1 = new Point3d (0.0, 0.0, 1.0);
      //PolygonalMesh box1 = MeshFactory.createBox (2.0, 1.0, 1.0, cen1, 16, 16, 16);
      //meshList.add (box1);
      //addBoxSurfaceMesh (4.0, 0.75, 0.75, 20, 5, 5);
      
      //addMesh (MeshExperiment.class, "geometry/tongue_surface_1.ply");
      //meshList.add (this.generatePointCloud (1, 10));
      //makeMesh ();
      //addRenderable (makeMB(meshList.get (0)));
      for (MeshBase mesh : meshList) {
         addRenderable (mesh);
      }
      setRenderProperties(meshList, Color.CYAN);
      myFFD = new NFFDController ();
      int [] degrees = {2, 2, 2};
      int [] numCPs = {6, 6, 6};
      myFFD.embedMeshes (meshList);
      myFFD.initializeNFFD (degrees, numCPs);
      addController (myFFD);
      myFFD.enableGridControl ();
   }
   
   public void addBoxSurfaceMesh (double wx, double wy, double wz, 
      int numX, int numY, int numZ) {
      Point3d center = new Point3d ();
      PolygonalMesh box = MeshFactory.createBox (
         wx, wy, wz, center, numX, numY, numZ);
      //MeshModelAgent agent = new MeshModelAgent ();
      //agent.represent (box);
      meshList.add (MeshModelAgent.makeMeshAgent (box));
   }

   public SkinMeshBody makeMB (MeshBase mesh) {
      SkinMeshBody mb = new SkinMeshBody ();
      mb.setMesh (mesh);
      return mb;
   }

   public FixedMeshBody makeFMB (MeshBase mesh) {
      FixedMeshBody mb = new FixedMeshBody ();
      mb.setMesh (mesh);
      return mb;
   }

   public void addMesh (Class<?> obj, String fileName) throws IOException {
      MeshBase mesh = ReadWrite.readMesh (obj, fileName);
      meshList.add (mesh);

   }

   public void setToRootRender() {
      for (MeshBase mesh : meshList) {
         getRoot (this).addRenderable (mesh);
      }
   }

   public void attach(DriverInterface driver) {
      driver.getViewerManager ().setBackgroundColor (Color.WHITE);
   }

   public PointMesh generatePointCloud (double length, int num) {
      PointMesh mesh = new PointMesh();
      double unit = length / (num-1);
      for (int i = 0; i < num; i++) {
         for (int j = 0; j < num; j++) {
            for (int k = 0; k < num; k++) {
               Point3d pnt3d = new Point3d (
                  i*unit, j*unit, k*unit);
               mesh.addVertex (pnt3d);
            }
         }
      }
      RigidTransform3d Xmw = new RigidTransform3d (
         length/2, length/2, length/2);
      mesh.inverseTransform (Xmw);

      return mesh;
   }



   //--------------------------------------------------
   // set render properties
   //--------------------------------------------------

   public static void setRenderProperties (ArrayList<MeshBase> argCom, Color argFaceColor) {
      for (MeshBase temMeshCom : argCom) {
         RenderProps.setFaceStyle (temMeshCom, FaceStyle.FRONT_AND_BACK);
         RenderProps.setFaceColor (temMeshCom, argFaceColor);
         RenderProps.setShading (temMeshCom, Shading.SMOOTH);
      }
   }

   public static void setRenderProperties (ArrayList<MeshBase> argCom, Color argFaceColor, Color argEdgeColor) {
      for (MeshBase temMeshCom : argCom) {
         RenderProps.setFaceStyle (temMeshCom, FaceStyle.FRONT_AND_BACK);
         RenderProps.setFaceColor (temMeshCom, argFaceColor);
         RenderProps.setShading (temMeshCom, Shading.SMOOTH);
         RenderProps.setDrawEdges (temMeshCom, true);
         RenderProps.setLineColor (temMeshCom, argEdgeColor);
      }
   }

   public static void setRenderProperties (ArrayList<MeshBase> argCom, String name, Color argFaceColor) {
      for (MeshBase temMeshCom : argCom) {
         if (temMeshCom.getName ().contains (name)) {
            RenderProps.setFaceStyle (temMeshCom, FaceStyle.FRONT_AND_BACK);
            RenderProps.setFaceColor (temMeshCom, argFaceColor);
            RenderProps.setShading (temMeshCom, Shading.SMOOTH);
            RenderProps.setDrawEdges (temMeshCom, false);
         }
      }
   }

   public static void setRenderProperties (ArrayList<MeshBase> argCom, String name, Color argFaceColor, Color argEdgeColor) {
      for (MeshBase temMeshCom : argCom) {
         if (temMeshCom.getName ().contains (name)) {
            RenderProps.setFaceStyle (temMeshCom, FaceStyle.FRONT_AND_BACK);
            RenderProps.setFaceColor (temMeshCom, argFaceColor);
            RenderProps.setShading (temMeshCom, Shading.SMOOTH);
            RenderProps.setDrawEdges (temMeshCom, true);
            RenderProps.setEdgeColor (temMeshCom, argEdgeColor);
         }
      }
   }

   public void makeMesh  () {
      MatrixNd data = new MatrixNd ();
      try {
         ReadWrite.readMatrix (data, ReadWrite.class, "/data/test.txt");
      }
      catch (IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
      PointMesh mesh = new PointMesh ();
      for(int i = 0; i < data.rowSize (); i++) {
         mesh.addVertex (data.get (i, 0), data.get (i, 1), data.get (i, 2));
      }
      meshList.add (mesh);
   }


}
