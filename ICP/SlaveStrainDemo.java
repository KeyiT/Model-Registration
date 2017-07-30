package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.FemModel3d;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;

public class SlaveStrainDemo extends ICPTest{

   public void build (String [] args) {
      addRenderable (fems);
      addRenderable (meshes);
      
      // source and target mesh
      Point3d cen1 = new Point3d (0.0, 0.0, 2.0);
      PolygonalMesh box1 = MeshFactory.createBox (3.0, 3.0, 3.0, cen1, 32, 32, 32);
      PolygonalMesh sphere1 = MeshFactory.createSphere (1.5, 24, 0.0, 0.0, 2.0);
      
      // slave
      addBeam (6.0, 1.0, 1.0, 20, 4, 4);
      
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
      icp.renderSourceAndTargetMesh (this);
      icp.createControlPanelForNFFDAction (this);
      icp.createRegistrationErrorProbe (this);
      
      icp.setFFDUpgradeRatio (0.06);
      icp.setSlaveStrainWeights (fems.get (0), 0.03);
      icp.upgradeFFD (1);
      icp.getCloudMap ().update ();
   }

}
