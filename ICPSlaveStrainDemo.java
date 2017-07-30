package artisynth.models.swallowingRegistrationTool;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.DriverInterface;
import artisynth.models.swallowingRegistrationTool.ICP.ICPManager;
import artisynth.models.swallowingRegistrationTool.ICP.ICPTest;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;

public class ICPSlaveStrainDemo extends ICPTest{

   public void build (String [] args) {
      addRenderable (fems);
      addRenderable (meshes);

      // source and target mesh
      Point3d cen1 = new Point3d (0.0, 0.0, 1.0);
      PolygonalMesh box1 = MeshFactory.createBox (2.0, 1.0, 1.0, cen1, 16, 16, 16);
      PolygonalMesh sphere1 = MeshFactory.createSphere (1.0, 24, 0.0, 0.0, 1.5);

      // slave
      addBeam (5.0, 0.75, 0.75, 15, 3, 3);
      //fems.get (0).transformPose (new RigidTransform3d (0, 0, -0.2));

      // initialize registration
      ICPManager icp = new ICPManager ("Demo");
      Map <MeshBase, MeshBase> map =  new HashMap <MeshBase, MeshBase> ();
      map.put (box1, sphere1);
      //map.put (box2, box22);
      //map.put (box3, box33);
      List<FemModel3d> slaves = new ArrayList<FemModel3d> ();
      slaves.addAll (fems);
      icp.initialize (map, slaves, null);
      addModel (icp);
      icp.renderSourceAndTargetMesh (this, 0.45, 0.5, 
         Shading.SMOOTH, Shading.SMOOTH);
      icp.createControlPanelForNFFDAction (this);
      icp.createRegistrationErrorProbe (this);

      icp.setEnableIteration (true);
      icp.setFFDUpgradeRatio (0.06);
      icp.setSlaveStrainWeights (fems.get (0), 0.03);
      icp.upgradeFFD (1);
      icp.getCloudMap ().update ();

      fems.get (0).setAutoGenerateSurface (true);
      fems.get (0).getSurfaceMesh ();
      fems.get (0).setSurfaceRendering (SurfaceRender.Shaded);

      RenderProps.setPointStyle (fems, PointStyle.SPHERE);
      RenderProps.setPointColor (fems, new Color (164, 44, 168));
      RenderProps.setPointRadius (fems, 0.02);
      RenderProps.setLineColor (fems, new Color (164, 41, 246));

      RenderProps.setFaceColor (fems.get (0).getSurfaceMeshComp (), 
         new Color (238, 232, 170));
      RenderProps.setAlpha(fems.get (0).getSurfaceMeshComp (), 
         0.5);

      /*
      ColorBar cBar = new ColorBar ();
      cBar.setName ("colorBar");
      cBar.setNumberFormat ("%.2f");
      cBar.populateLabels (0.0, 1.0, 10);
      cBar.setLocation (0.9, 0.1, 0.1, 0.8);
      addRenderable (cBar);*/
   }


   public void attach (DriverInterface driver)
   {
      
      this.getMainViewer().setBackgroundColor(Color.white);

      getMainViewer ().setEye (new Point3d (-1.73941, -10.4438, 1.08766));
      //getMainViewer ().setEye (new Point3d (-5.42821, -10.1769, 3.52089));
      getMainViewer ().setTransparencyFaceCulling (true);
      getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      getMainViewer ().setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
    
   }

   @Override
   public StepAdjustment advance (
      double t0, double t1, int flags) {
      StepAdjustment sa = super.advance (t0, t1, flags);
      fems.get (0).updateSlavePos ();
      //fems.get (0).updateStressAndStiffness ();
      return sa;
   }

   
   public void prerender (RenderList list) {
      super.prerender (list);

      // Synchronize color bar/values in case they are changed
      /*
      ColorBar cbar = (ColorBar)(renderables().get("colorBar"));

      cbar.setColorMap(fems.get (0).getColorMap());

      DoubleInterval range = fems.get (0).getStressPlotRange();

      cbar.updateLabels(range.getLowerBound(), range.getUpperBound());*/
   }
}
