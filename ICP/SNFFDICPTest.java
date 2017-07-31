package artisynth.models.swallowingRegistrationTool.ICP;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.models.swallowingRegistrationTool.main.IterativeActionsPanel;
import artisynth.models.swallowingRegistrationTool.main.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.infoUtilities.UniformMeshFeatureSubsampler;
import artisynth.models.swallowingRegistrationTool.infoUtilities.MeshInfoAllocator.MeshFeature;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.QuadraticBasedLinearOptimizer;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.utilities.FemModelAgent;
import maspack.geometry.PolygonalMesh;

public class SNFFDICPTest extends ICPTest{

   @Override
   public void buildRegistration (FemModel3d fem1, FemModel3d fem2) {
      FemModelAgent srcAgent = new FemModelAgent (fem1);
      FemModelAgent tgtAgent = new FemModelAgent (fem2);


      PolygonalMesh src = srcAgent.regenerateSurfaceMeshAgent ();
      PolygonalMesh tgt = tgtAgent.regenerateSurfaceMeshAgent ();

      RegistrationManager icp = new RegistrationManager ("ICP");
      // ----------------------------registration initialization start---------------------------- //
      // set mesh
      icp.addMeshMatchPair (src, tgt);
      icp.renderSourceAndTargetMesh (this);

      // add action
      AffineModelTransformer affine = new AffineModelTransformer ();
      affine.getClass ();
      affine.addMasterMesh (src);
      icp.addAction (affine, new ARUNSVDOptimizer());
      NFFDeformer ffd = new NFFDeformer (2, 1, src);
      icp.addAction (ffd, new QuadraticBasedLinearOptimizer());
      
      icp.enableSlaveElementSaving (true);
      icp.enableSlaveElementWarping (true);
      icp.enableSlaveStiffnessSaving (true);
      ffd.enableSlaveBTKSaving (true);
      ffd.enableSlaveElementBTKSaving (true);
      ffd.enableSlaveFFDPointSaving (true);
      icp.addSlave (fem1);
      icp.assignSlaveInfo ();

      // add map
      icp.setCloudMap (new ClosestPoint3dMap());

      // build allocator
      UniformMeshFeatureSubsampler allo1 = new UniformMeshFeatureSubsampler ();
      allo1.addMesh (src);
      allo1.setTargetMesh (tgt);
      allo1.setAllocateFeature (MeshFeature.Vertex);
      allo1.setDirection (true);

      UniformMeshFeatureSubsampler allo2 = new UniformMeshFeatureSubsampler ();
      allo2.addMesh (tgt);
      allo2.setTargetMesh (src);
      allo2.setAllocateFeature (MeshFeature.Vertex);
      allo2.setDirection (false);
      UniformMeshFeatureSubsampler[] allos = new UniformMeshFeatureSubsampler [2];
      allos[0] = allo1;
      allos[1] = allo2;

      // allocate correspondence
      try {
         icp.allocateCorrespondences (allos);
      }
      catch (ReflectiveOperationException e) {
         System.err.println ("Failed to allocator correspondence");
         e.printStackTrace();
      }

      // set iteration actions
      SNFFDICPIterativeActions myNFFDIAs = new SNFFDICPIterativeActions ();
      AffineICPIterativeAction myAffineIAs = new AffineICPIterativeAction ();
      IterativeActionsPanel myIAs = new IterativeActionsPanel ();
      myIAs.addIterativeActions (myAffineIAs);
      myIAs.addIterativeActions (myNFFDIAs);
      icp.setIterativeAction (myIAs);
      // ----------------------------registration initialization end---------------------------- //

      addModel (icp);
      ControlPanel panel = icp.createRegistrationControlPanel ();
      myNFFDIAs.controlMapDirection (panel, this);
      myNFFDIAs.controlFFD (ffd, panel, this);
      myIAs.addWidgetsToExternalPanel (panel, this);
      icp.createRegistrationErrorProbe (this);
      addControlPanel (panel);
   }

}
