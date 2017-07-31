package artisynth.models.swallowingRegistrationTool.demo;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.models.swallowingRegistrationTool.ICP.ICPManager;
import artisynth.models.swallowingRegistrationTool.ICP.ICPTest;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.Renderer.Shading;

public class ICPSlaveLaplacianDemo extends ICPTest{
   
   MeshComponentList <MeshModelAgent> slaveMeshes = 
   new MeshComponentList <MeshModelAgent> (MeshModelAgent.class);

   public void build (String [] args) {
      addRenderable (slaveMeshes);
      addRenderable (meshes);

      // source and target mesh
      Point3d cen1 = new Point3d (0.0, 0.0, 1.0);
      PolygonalMesh box1 = MeshFactory.createBox (2.0, 1.0, 1.0, cen1, 6, 6, 6);
      PolygonalMesh sphere1 = MeshFactory.createSphere (1.0, 24, 0.0, 0.0, 1.5);

      // slave
      addBoxSurfaceMesh (5.0, 0.75, 0.75, 16, 3, 3);
      //slaveMeshes.get (0).transformMesh (new RigidTransform3d (0, 0, -0.2));

      // initialize registration
      ICPManager icp = new ICPManager ("Demo");
      Map <MeshBase, MeshBase> map =  new HashMap <MeshBase, MeshBase> ();
      map.put (box1, sphere1);
      //map.put (box2, box22);
      //map.put (box3, box33);
      List <MeshModelAgent> slaves = new ArrayList <MeshModelAgent> ();
      slaves.addAll (slaveMeshes);
      icp.initialize (map, null, slaves);
      addModel (icp);
      icp.renderSourceAndTargetMesh (this, 0.45, 0.5, 
         Shading.SMOOTH, Shading.SMOOTH);
      icp.createControlPanelForNFFDAction (this);
      icp.createRegistrationErrorProbe (this);

      icp.setEnableIteration (true);
      icp.setFFDUpgradeRatio (0.06);
      icp.setSlaveBendingWeights (slaveMeshes.get (0), 0);
      icp.setSlaveARAPWeights (slaveMeshes.get (0), 0.0);
      icp.setSlaveACAPWeights (slaveMeshes.get (0), 0.16);
      icp.upgradeFFD (0);
      icp.getCloudMap ().update ();
      icp.enableSlaveConformalModesUpdate (false);
      
      slaveMeshes.getRenderProps ().setDrawEdges (true);
      slaveMeshes.getRenderProps ().setEdgeColor (new Color (255, 102, 102));
      slaveMeshes.getRenderProps ().setFaceColor (new Color (238, 232, 170));
      slaveMeshes.getRenderProps ().setShading (Shading.SMOOTH);
   }
   
   public void addBoxSurfaceMesh (double wx, double wy, double wz, 
      int numX, int numY, int numZ) {
      Point3d center = new Point3d ();
      PolygonalMesh box = MeshFactory.createBox (
         wx, wy, wz, center, numX, numY, numZ);
      MeshModelAgent agent = new MeshModelAgent ();
      agent.represent (box);
      slaveMeshes.add (agent);
   }
   
   @Override
   public StepAdjustment advance (
      double t0, double t1, int flags) {
      ((PolygonalMesh)slaveMeshes.get (0).getMesh ()).notifyVertexPositionsModified ();
      ((PolygonalMesh)slaveMeshes.get (0).getMesh ()).updateRenderNormals ();
      return super.advance (t0, t1, flags);
   }
}
