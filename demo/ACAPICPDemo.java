package artisynth.models.swallowingRegistrationTool.demo;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.ICP.ICPACAPManager;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

public class ACAPICPDemo extends RootModel{
   
   ICPACAPManager icp = new ICPACAPManager ("Demo");
   List <MeshModelAgent> slaves = new ArrayList <MeshModelAgent> ();
   
   public void build (String [] args) {
      
      PolygonalMesh tube = MeshFactory.createTube (0.6, 1.0, 1.3, 20, 20, 10);
      PolygonalMesh torus = MeshFactory.createTorus (0.6, 0.3, 20, 16);
      torus.transform (new RigidTransform3d (2.5, 0, 0, 0, 0, 1, Math.PI));
      
      /*
      PolygonalMesh sphere = MeshFactory.createSphere (0.8, 16, 2.5, -2.5, 0.0);
      PolygonalMesh box = MeshFactory.createBox (
         1.0, 1.5, 1.0, 
         new Point3d (0, -2.5, 0), 
         10, 15, 10);*/
      
      
      // initialize registration
      Map <PolygonalMesh, MeshBase> map =  new HashMap <PolygonalMesh, MeshBase> ();
      map.put (torus, tube);
      //map.put (sphere, box);

      
      MeshModelAgent slave = new MeshModelAgent ();
      slave.represent (torus);
      slaves.add (slave);
      icp.initialize (map);
      addModel (icp);
      icp.renderSourceAndTargetMesh (this);
      
      RenderProps srcProp = new RenderProps ();
      srcProp.setFaceColor (Color.RED);
      srcProp.setFaceStyle (FaceStyle.FRONT_AND_BACK);
      srcProp.setShading (Shading.SMOOTH);
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
      icp.createRegistrationErrorProbe (this);
      icp.setRigidity (0.6);

      icp.setEnableIteration (true);
   }
   
   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);

      getMainViewer ().setEye (new Point3d (-1.06606, -5.35973, 7.72179));
      getMainViewer ().setCenter (new Point3d (1.2, 0, -5.55112e-17));
      getMainViewer ().setTransparencyFaceCulling (true);
      getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      getMainViewer ().setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
   }
}