package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.SkinMeshBody;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.Point;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.FemModelDeformer;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank3.FrankModel3;
import artisynth.models.modelOrderReduction.PrintData;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.swallowingRegistrationTool.transformers.BezierFFD3d.ControlPoint;
import artisynth.models.swollowingFrank.FrankReg;
import maspack.geometry.MeshBase;
import maspack.geometry.PointMesh;
import maspack.geometry.MeshBase;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.PropertyList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

public class BezerFFD3dTest extends RootModel{

   MechModel mechModel = new MechModel ("Test");
   ArrayList<MeshBase> meshList = new ArrayList<MeshBase> ();
   protected BezierFFD3d myFFD;

   // ------------------------- properties ------------------------------//
   static PropertyList myProps = new PropertyList (BezerFFD3dTest.class, RootModel.class);
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add ("printJac", "print jacobian determinante", false);
   }

   public void setPrintJac(boolean print) {
      if (print) {
         this.printJacDet ();
      }
   }

   public boolean getPrintJac() {
      return false;
   }

   ControlPanel myPanel = new ControlPanel();


   public void build(String [] args) throws IOException {
      super.build (args);
      addModel (mechModel);

      //addMesh (FrankModel3.class, "softPalate/palateSurface_rst.obj");
      //addMesh (FrankModel3.class, "C0.obj");
      //addMesh (FrankReg.class, "tongue_surface_1.ply");
      meshList.add (this.generatePointCloud (1, 10));
      mechModel.add (makeMB(meshList.get (0)));
      setRenderProperties(meshList, Color.CYAN);
      myFFD = new BezierFFD3d(3, 3, 3, meshList);
      addModel(myFFD);
      addControlPanel(myFFD.createControlPanel ());

      myPanel.addWidget (this, "printJac");
      addControlPanel(myPanel);
      //FemModelDeformer deformer =
      //new FemModelDeformer ("deformer", this, 2);
      //addModel (deformer);
      //addControlPanel (deformer.createControlPanel());
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

   public void printJacDet () {
      VectorNd dets = new VectorNd (myFFD.getCtPntNum ());
      double det = 0;
      int i = 0;
      Vector3d cen = new Vector3d();
      for (ControlPoint pnt : myFFD.getControlPoints ()) {
         det = myFFD.computeJacobian1 (pnt.getUndeformedPosition ()).determinant ();
         dets.set (i, det);
         i++;
      }
      cen.add (myFFD.getControlPoint (0).getUndeformedPosition ());
      cen.add (myFFD.getControlPoint (17).getUndeformedPosition ());
      cen.scale (0.5);
      PrintData.printVector (dets);
      System.out.println (myFFD.computeJacobian1 (cen).determinant ());
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

}
