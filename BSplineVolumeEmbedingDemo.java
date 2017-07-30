package artisynth.models.swallowingRegistrationTool;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDController;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;

public class BSplineVolumeEmbedingDemo extends RootModel{

   ArrayList<MeshBase> meshList = new ArrayList<MeshBase> ();
   protected NFFDController myFFD;

   ControlPanel myPanel = new ControlPanel();

   protected RenderProps controlPointsRenderProps = new RenderProps ();
   
   FemModel3d myFem;


   public void build(String [] args) throws IOException {
      super.build (args);
      
      PolygonalMesh mesh = MeshFactory.createCylinder (0.25, 1, 30, 10, 30);
      
      int [] degrees = {1, 1, 1};
      int [] ctlPnts = {3, 3, 10};
      myFFD = new NFFDController ();
      myFFD.initializeNFFD (ctlPnts, degrees, 
         new RigidTransform3d (-0.283299, -0.275203, -0.501056), 
         new Point3d (0.283299, 0.275203, 0.501056));
      myFFD.embedMesh (mesh);
      myFFD.getFFD ().advanceMeshInitialStates ();
      
      addController (myFFD);
      myFFD.enableGridControl ();
      addRenderable (mesh);
      
      RenderProps.setFaceColor (mesh, Color.RED);
      RenderProps.setShading (mesh, Shading.SMOOTH);
      RenderProps.setFaceStyle (mesh, FaceStyle.FRONT_AND_BACK);
      
      
      MechModel mechModel = new MechModel ();
      myFem = makeFEM();
      for (FemNode3d node : myFem.getNodes ()) {
         if (Math.abs (node.getPosition ().z) > 0.5) {
            node.setDynamic (false);
         }
         node.setMass (0);
      }
      mechModel.addModel (myFem);
      addModel (mechModel);
      
      myFem.setSurfaceRendering (SurfaceRender.Shaded);
      myFem.getRenderProps ().setFaceColor (new Color (204, 204, 204));
      myFem.getRenderProps ().setShading (Shading.SMOOTH);
      myFem.getRenderProps ().setLineColor (new Color (0, 153, 255));
      myFem.getRenderProps ().setPointStyle (PointStyle.SPHERE);
      myFem.getRenderProps ().setPointRadius (0.01);
      myFem.getRenderProps ().setPointColor (new Color (0, 102, 204));
      
      NodeController ct = new NodeController (this);
      addController (ct);
   }
   
   
   
   public FemModel3d makeFEM () {
      
      int [] nums = myFFD.getFFD ().getCtrlPntsNum ();

      FemModel3d fem = new FemModel3d ();
      VectorNd rest = new VectorNd (myFFD.getFFD ().numControlPoints ()*3);

      for (int i = 0; i < myFFD.getFFD ().numControlPoints (); i++) {
         Point3d pnt = myFFD.getFFD ().getCtrlPointRestPosition (i);
         fem.addNode (new FemNode3d (pnt));
         rest.setSubVector (i*3, pnt);
      }

      ComponentListView<FemNode3d> nodes = fem.getNodes();

      int wk = (nums[2]) * (nums[1]);
      int wj = (nums[2]);
      boolean flag = true;

      HexElement ee =
      new HexElement(
         nodes.get(0),
         nodes.get(1), 
         nodes.get( wj + 1), 
         nodes.get( wj ),

         nodes.get(wk), 
         nodes.get(wk +  1), 
         nodes.get( wk +  wj + 1), 
         nodes.get( wk +  wj ));
      if (ee.isInvertedAtRest ()) {
         flag = false;
      }

      for (int i = 0; i < nums[2]-1; i++) {
         for (int j = 0; j < nums[1]-1; j++) {
            for (int k = 0; k < nums[0]-1; k++) {
               HexElement e = null;
               if (flag) {
                  e =
                  new HexElement(
                     nodes.get(k * wk + j * wj + i),
                     nodes.get(k * wk + j * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i),

                     nodes.get((k + 1) * wk + j * wj + i), 
                     nodes.get((k + 1) * wk + j * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i));
               }
               else {
                  e =
                  new HexElement(

                     nodes.get((k + 1) * wk + j * wj + i), 
                     nodes.get((k + 1) * wk + j * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i),

                     nodes.get(k * wk + j * wj + i),
                     nodes.get(k * wk + j * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i));
               }

               e.setParity((i + j + k) % 2 == 0 ? 1 : 0);

               fem.addElement(e);
            }
         }
      }
      fem.invalidateStressAndStiffness ();
      fem.setLinearMaterial (2E+5, 0.3, false);
      fem.resetRestPosition ();
      
      return fem;
   }

   public void attach(DriverInterface driver) {
      driver.getViewerManager ().setBackgroundColor (Color.WHITE);
      
      this.getMainViewer ().setEye (new Point3d (-1.96336, -2.6928, 0.90006));
      this.getMainViewer ().setCenter (new Point3d (-5.55112e-17, 0, 0.25));
      this.getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      this.getMainViewer ().setBlendDestFactor (BlendFactor.GL_SRC_ALPHA);
      this.getMainViewer ().setTransparencyFaceCulling (true);
   }
   
   public StepAdjustment advance (
      double t0, double t1, int flags) {
      
      StepAdjustment ad = super.advance (t0, t1, flags);
      
      for (int i = 0; i < myFem.numNodes (); i++) {
         FemNode3d node=  myFem.getNode (i);
         myFFD.getFFD ().setCtrlPntPosition (i, node.getPosition ());
      }
      return ad;
   }
   
   public class NodeController extends ControllerBase {
      
      BSplineVolumeEmbedingDemo myDemo;
      FemModel3d myFem;
      ArrayList<FemNode3d> nodes = new ArrayList<FemNode3d> ();
      ArrayList<FemNode3d> nodes1 = new ArrayList<FemNode3d> ();
      ArrayList<FemNode3d> nodes2 = new ArrayList<FemNode3d> ();
      
      public NodeController (BSplineVolumeEmbedingDemo demo) {
         myDemo = demo;
         myFem = myDemo.myFem;
         for (FemNode3d node: myFem.getNodes ()) {
            if (node.getPosition ().z > 0.5) {
               nodes.add (node);
            }
            if (Math.abs (node.getPosition ().z) < 0.07 
            && node.getPosition ().x > 0.25) {
               nodes1.add (node);
            }
            if (Math.abs (node.getPosition ().z) < 0.07 
            && node.getPosition ().x < -0.25) {
               nodes2.add (node);
            }
         }
      }

      double myT0 = 1.0;
      
      @Override
      public void apply (double t0, double t1) {
         
         if (t0 < myT0) {
            RigidTransform3d rt = new RigidTransform3d (0, 0, (t1-t0) / myT0 * 0.5);
            for (FemNode3d node : nodes) {
               node.getPosition ().transform (rt);
            }
         }
         else if (t0 < 2 * myT0) {
            for (FemNode3d node : nodes1) {
               node.setDynamic (false);
            }
            for (FemNode3d node : nodes2) {
               node.setDynamic (false);
            }
            
            RigidTransform3d rt = new RigidTransform3d ( (t1-t0) / myT0 * 0.3, 0, 0);
            for (FemNode3d node : nodes1) {
               node.getPosition ().transform (rt);
            }
            rt = new RigidTransform3d (- (t1-t0) / myT0 * 0.3, 0, 0);
            for (FemNode3d node : nodes2) {
               node.getPosition ().transform (rt);
            }
         }
      }
      
   }
   
}