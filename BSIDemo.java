package artisynth.models.swallowingRegistrationTool;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.subjectFrank.SubjectModel;
import artisynth.models.swallowingRegistrationTool.BSI.BSIManager;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;

public class BSIDemo extends RootModel{

   BSIManager bsi = new BSIManager ("Demo");
   protected RenderableComponentList <FemModel3d> fems =
   new RenderableComponentList <FemModel3d> (FemModel3d.class);
   protected RenderableComponentList<MeshModelAgent> meshes = 
   new RenderableComponentList<MeshModelAgent> (MeshModelAgent.class);

   public void build (String [] args) {

      addRenderable (fems);
      addRenderable (meshes);

      // source and target mesh
      Point3d cen1 = new Point3d (0.0, 0.0, 1.0);
      PolygonalMesh box1 = MeshFactory.createBox (2.0, 1.0, 1.0, cen1, 16, 16, 16);
      PolygonalMesh sphere1 = SubjectModel.readMeshWithoutSuffix (
         ArtisynthPath.getSrcRelativePath (BSIDemo.class, "geometry/box2sphere"));

      // slave
      addBeam (5.0, 0.75, 0.75, 16, 5, 5);
      fems.get (0).transformPose (new RigidTransform3d (0, 0, 0.02));

      // initialize registration
      Map <MeshBase, MeshBase> map =  new HashMap <MeshBase, MeshBase> ();
      map.put (box1, sphere1);
      //map.put (box2, box22);
      //map.put (box3, box33);
      List<FemModel3d> slaves = new ArrayList<FemModel3d> ();
      slaves.addAll (fems);
      bsi.initialize (map, slaves, null);
      addModel (bsi);
      bsi.renderSourceAndTargetMesh (this, 0.45, 0.5, 
         Shading.SMOOTH, Shading.SMOOTH);
      bsi.createControlPanel (this);
      bsi.createRegistrationErrorProbe (this);

      bsi.setEnableIteration (true);
      bsi.setFFDUpgradeRatio (0.08);
      bsi.setSlaveStrainWeights (fems.get (0), 0.03);
      bsi.upgradeFFD (0);
      bsi.getCloudMap ().update ();
      bsi.enableNFFDIterativeActions ();

      RenderProps srcProp = new RenderProps ();
      srcProp.setFaceColor (Color.RED);
      srcProp.setFaceStyle (FaceStyle.FRONT_AND_BACK);
      srcProp.setShading (Shading.FLAT);
      //srcProp.setDrawEdges (true);
      //srcProp.setEdgeColor (Color.BLACK);

      RenderProps tgtProp = new RenderProps ();
      tgtProp.setFaceColor (Color.BLUE);
      tgtProp.setFaceStyle (FaceStyle.FRONT);
      tgtProp.setShading (Shading.FLAT);
      tgtProp.setEdgeColor (Color.BLACK);
      tgtProp.setAlpha (0.5);

      bsi.setSourceMeshRenderProps (srcProp);
      bsi.setTargetMeshRenderProps (tgtProp);

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
   
   public void addBeam (double wx, double wy, double wz, 
      int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createHexGrid (beam, wx, wy, wz, numX, numY, numZ);
      fems.add (beam);
   }

}
