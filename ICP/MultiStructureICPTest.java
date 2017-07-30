package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.HashMap;
import java.util.Map;

import artisynth.models.swallowingRegistrationTool.utilities.FemModelAgent;
import maspack.geometry.MeshBase;
import maspack.matrix.RigidTransform3d;

public class MultiStructureICPTest extends ICPTest{

   public void build (String [] args) {
      addRenderable (fems);
      addRenderable (meshes);
      
      addBeam (0.2, 0.2, 2, 10, 10, 10);
      addBeam (0.2, 1, 0.2, 5, 5, 5);
      fems.get (1).transformGeometry (
         new RigidTransform3d (0, -0.6, 0));
      
      addEllipsoid (1.4, 0.2, 0.2, 10, 10, 10);
      addBeam (0.2, 0.7, 0.2, 6, 6, 6);
      fems.get (3).transformGeometry (
         new RigidTransform3d (
            0, -0.3, 0.1, 1.0, 0.0, 0.0, Math.toRadians (-30)));
      
      Map <MeshBase, MeshBase> meshMap = 
      new HashMap <MeshBase, MeshBase> ();
      
      FemModelAgent agent0 = new FemModelAgent ("srcFem");
      agent0.represent (fems.get (0));
      FemModelAgent agent1 = new FemModelAgent ("tgtFem");
      agent1.represent (fems.get (2));
      meshMap.put (agent0.regenerateSurfaceMeshAgent (), 
         agent1.regenerateSurfaceMeshAgent ());
      
      agent0 = new FemModelAgent ("srcFem");
      agent0.represent (fems.get (1));
      agent1 = new FemModelAgent ("tgtFem");
      agent1.represent (fems.get (3));
      meshMap.put (agent0.regenerateSurfaceMeshAgent (), 
         agent1.regenerateSurfaceMeshAgent ());
      
      
      ICPManager manager = new ICPManager ("ICP");
      manager.initialize (meshMap, null, null);
      addModel (manager);
      manager.createControlPanel (this);
      manager.renderSourceAndTargetMesh (this);
   }
   
   
}
