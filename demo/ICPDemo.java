package artisynth.models.swallowingRegistrationTool.demo;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.modelbase.ControllerBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.ICP.ICPACAPManager;
import artisynth.models.swallowingRegistrationTool.ICP.ICPARAPManager;
import artisynth.models.swallowingRegistrationTool.ICP.ICPManager;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

public class ICPDemo extends RootModel{

   ICPACAPManager icp = new ICPACAPManager ("Demo");
   List <MeshModelAgent> slaves = new ArrayList <MeshModelAgent> ();

   public void build (String [] args) {


      // source and target mesh
      Point3d cen1 = new Point3d (0.0, 0.0, 1.0);
      PolygonalMesh box = MeshFactory.createBox (2.0, 1.0, 1.0, cen1, 16, 16, 16);
      PolygonalMesh sphere = MeshFactory.createSphere (1.0, 24, 0.0, 0.0, 1.5);


      // initialize registration
      Map <PolygonalMesh, MeshBase> map =  new HashMap <PolygonalMesh, MeshBase> ();
      map.put (box, sphere);


      MeshModelAgent slave = new MeshModelAgent ();
      slave.represent (box);
      slaves.add (slave);
      icp.initialize (map);
      addModel (icp);
      icp.renderSourceAndTargetMesh (this, 0.45, 0.5, 
         Shading.SMOOTH, Shading.SMOOTH);

      RenderProps srcProp = new RenderProps ();
      srcProp.setFaceColor (Color.RED);
      srcProp.setFaceStyle (FaceStyle.FRONT_AND_BACK);
      srcProp.setShading (Shading.FLAT);
      //srcProp.setDrawEdges (true);
      //srcProp.setEdgeColor (Color.BLACK);

      RenderProps tgtProp = new RenderProps ();
      tgtProp.setFaceColor (Color.CYAN);
      tgtProp.setFaceStyle (FaceStyle.FRONT_AND_BACK);
      tgtProp.setShading (Shading.FLAT);
      tgtProp.setEdgeColor (Color.BLACK);
      tgtProp.setAlpha (0.5);

      icp.setSourceMeshRenderProps (srcProp);
      icp.setTargetMeshRenderProps (tgtProp);

      icp.createControlPanelForMLDAction (this);
      icp.setEnableIteration (true);
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

}

