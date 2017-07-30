package artisynth.models.swallowingRegistrationTool.ICP;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.frank2.GenericModel;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.swallowingRegistrationTool.IterativeActionsPanel;
import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.VertexToMeshCorrespondence;
import artisynth.models.swallowingRegistrationTool.infoUtilities.MeshInfoAllocator.MeshFeature;
import artisynth.models.swallowingRegistrationTool.infoUtilities.UniformMeshFeatureSubsampler;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadraticLinearOptimizer;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.utilities.FEMQualityUtilities;
import artisynth.models.swallowingRegistrationTool.utilities.FemModelAgent;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.GL.GLViewer.BlendFactor;

public class ICPTest extends RootModel{

   protected RenderableComponentList <FemModel3d> fems =
   new RenderableComponentList <FemModel3d> (FemModel3d.class);
   protected RenderableComponentList<MeshModelAgent> meshes = 
   new RenderableComponentList<MeshModelAgent> (MeshModelAgent.class);

   public void build (String [] args) {

      addRenderable (fems);
      addRenderable (meshes);

      //addPyramidSquareBeam(1, 5, 5, 5);
      //addWedgeSquareBeam(1, 5, 5, 5);
      //addTetSquareBeam (1, 5, 5, 5);
      addSquareBeam (0.8, 5, 5, 5);
      addEllipsoid (2.5, 0.6, 0.6, 6, 6, 6);
      fems.get (1).getRenderProps ().setVisible (false);
      addController (new FemController (fems.get (0)));

      buildRegistration (fems.get (0), fems.get (1));
   }

   public void buildRigidRegistration (FemModel3d fem1, FemModel3d fem2) {
      FemModelAgent srcAgent = new FemModelAgent (fem1);
      FemModelAgent tgtAgent = new FemModelAgent (fem2);


      PolygonalMesh src = srcAgent.regenerateSurfaceMeshAgent ();
      PolygonalMesh tgt = tgtAgent.regenerateSurfaceMeshAgent ();

      RegistrationManager icp = new RegistrationManager ("Affine");
      // ----------------------------registration initialization start---------------------------- //
      // set mesh
      icp.addMeshMatchPair (src, tgt);
      icp.renderSourceAndTargetMesh (this);

      // add action
      AffineModelTransformer affine = new AffineModelTransformer ();
      affine.addMasterMesh (src);
      icp.addAction (affine, new ARUNSVDOptimizer());
      
      // add slave
      icp.addSlave (fem1);
      icp.assignSlaveInfo ();

      // add map
      icp.setCloudMap (new ClosestPoint3dMap<VertexToMeshCorrespondence>());

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

      // set iteration action
      icp.setIterativeAction (new AffineICPIterativeAction());
      // ----------------------------registration initialization end---------------------------- //

      addModel (icp);
      ControlPanel panel = icp.createRegistrationControlPanel ();
      icp.createRegistrationErrorProbe (this);
      addControlPanel (panel);
   }

   public void buildNFFDRegistration (FemModel3d fem1, FemModel3d fem2) {
      FemModelAgent srcAgent = new FemModelAgent (fem1);
      FemModelAgent tgtAgent = new FemModelAgent (fem2);


      PolygonalMesh src = srcAgent.regenerateSurfaceMeshAgent ();
      PolygonalMesh tgt = tgtAgent.regenerateSurfaceMeshAgent ();

      RegistrationManager icp = new RegistrationManager ("NFFD");
      // ----------------------------registration initialization start---------------------------- //
      // set mesh
      icp.addMeshMatchPair (src, tgt);
      icp.renderSourceAndTargetMesh (this);

      // add action
      NFFDeformer ffd = new NFFDeformer (5, 2, src);
      icp.addAction (ffd, new SparseQuadraticLinearOptimizer());
      
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
      NFFDICPIterativeActions myIAs = new NFFDICPIterativeActions ();
      icp.setIterativeAction (myIAs);
      // ----------------------------registration initialization end---------------------------- //

      addModel (icp);
      ControlPanel panel = icp.createRegistrationControlPanel ();
      myIAs.controlMapDirection (panel, this);
      myIAs.controlFFD (ffd, panel, this);
      icp.createRegistrationErrorProbe (this);
      addControlPanel (panel);
   }
   
  

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
      icp.addAction (ffd, new SparseQuadraticLinearOptimizer());
      
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
      NFFDICPIterativeActions myNFFDIAs = new NFFDICPIterativeActions ();
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


   public class FemController extends ControllerBase {
      FemModel3d myFem;
      FEMQualityUtilities femQU = new FEMQualityUtilities ();
      public FemController (FemModel3d fem) {
         myFem = fem;
      }

      @Override
      public void apply (double t0, double t1) {
         femQU.renderMeanRatioForElements (myFem);
      }

   }

   public PolygonalMesh readMesh (String name) {
      PolygonalMesh mesh = null;
      try {
         mesh = ReadWrite.readMesh (getClass(), "data/" + name);
      }
      catch (IOException e) {
         System.out.println ("Failed to read mesh file!");
         e.printStackTrace();
         System.exit (1);
      }
      return mesh;
   }

   public FemModel3d addFem (String name) {
      FemModel3d fem = new FemModel3d ();
      String dataDir = 
      ArtisynthPath.getSrcRelativePath (ICPManager.class, "data/");
      GenericModel.loadFemMesh_VTK (fem, dataDir, name);
      fem.setName (name);
      FemModelAgent agent = new FemModelAgent(fem);
      fems.add (agent);
      return agent;
   }


   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);

      getMainViewer ().setEye (new Point3d (0, -5.30116, 0));
      //getMainViewer ().setEye (new Point3d (-5.42821, -10.1769, 3.52089));
      //getMainViewer ().setEye (new Point3d (-1.73941, -10.4438, 1.08766));
      getMainViewer ().setTransparencyFaceCulling (true);
      getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      getMainViewer ().setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
   }
   
   public void addBeam (double wx, double wy, double wz, 
      int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createHexGrid (beam, wx, wy, wz, numX, numY, numZ);
      fems.add (beam);
   }

   public void addSquareBeam (double width, int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createHexGrid (beam, width, width, width, numX, numY, numZ);
      fems.add (beam);
   }

   public void addTetSquareBeam (double width, int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createTetGrid (beam, width, width, width, numX, numY, numZ);
      fems.add (beam);
   }

   public void addWedgeSquareBeam (double width, int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createWedgeGrid (beam, width, width, width, numX, numY, numZ);
      fems.add (beam);
   }

   public void addPyramidSquareBeam (double width, int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createPyramidGrid (beam, width, width, width, numX, numY, numZ);
      fems.add (beam);
   }

   public void addEllipsoid (double rl, double rs2, double rs3, int n1, int n2, int n3) {
      FemModel3d ell = new FemModel3d ();
      FemFactory.createEllipsoid (ell, rl/2.0, rs2/2.0, rs3/2.0, n1, n2, n3);
      fems.add (ell);
   }

   public static void main (String [] args) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createHexGrid (beam, 1, 1, 1, 3, 3, 3);
      //System.out.println (beam.getActiveStiffness ());

   }

}
