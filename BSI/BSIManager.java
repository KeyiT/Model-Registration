package artisynth.models.swallowingRegistrationTool.BSI;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.IterativeActionsPanel;
import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.correspondences.SurfaceFixedPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.VertexToPointCorrespondence;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.TransformerSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.UniformMeshFeatureSubsampler;
import artisynth.models.swallowingRegistrationTool.infoUtilities.MeshInfoAllocator.MeshFeature;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadraticLinearOptimizer;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDSlavePanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.matrix.ImproperStateException;
import maspack.properties.PropertyList;

public class BSIManager extends RegistrationManager{

   IterativeActionsPanel myIAs = new IterativeActionsPanel ();
   NFFDTIMIterativeActions myNFFDIAs = new NFFDTIMIterativeActions ();
   AffineTIMIterativeActions myAffineIAs = new AffineTIMIterativeActions ();

   NFFDeformer ffd;
   AffineModelTransformer affine;

   SurfaceFixedPoint3dMap<VertexToPointCorrespondence> map;

   //--------------------------implements properties-----------------------//

   public static PropertyList myProps =
   new PropertyList (BSIManager.class, RegistrationManager.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
   }


   public BSIManager (String name) {
      super (name);
   }

   public void initialize (Map<MeshBase, MeshBase> Src2TgtMap,
      List<FemModel3d> femSlaves, List<MeshModelAgent> meshSlaves) {

      addMeshMatchPairs (Src2TgtMap);

      // add action
      affine = new AffineModelTransformer ();
      affine.getClass ();
      affine.addMasterMeshes (Src2TgtMap.keySet ());
      addAction (affine, new ARUNSVDOptimizer());
      ffd = new NFFDeformer (2, 1, Src2TgtMap.keySet ());
      addAction (ffd, new SparseQuadraticLinearOptimizer());

      // add slaves
      if (femSlaves != null) {
         enableSlaveElementSaving (true);
         enableSlaveElementWarping (true);
         enableSlaveStiffnessSaving (true);
         enableConformalModesUpdate (false);
         enableElementStiffnessSaving (true);
         ffd.enableSlaveBTKSaving (true);
         ffd.enableSlaveElementBTKSaving (true);
         ffd.enableSlaveFFDPointSaving (true);
         for (FemModel3d slave : femSlaves) {
            addSlave (slave);
            assignSlaveInfo (slave);
         }
      }
      if (meshSlaves != null) {
         enableSlaveElementSaving (false);
         enableSlaveElementWarping (false);
         enableSlaveStiffnessSaving (false);
         enableConformalModesUpdate (false);
         enableElementStiffnessSaving (false);
         enableSlaveEdgeSaving (true);
         ffd.enableSlaveBTKSaving (false);
         ffd.enableSlaveElementBTKSaving (false);
         ffd.enableSlaveFFDPointSaving (true);
         for (MeshModelAgent slave : meshSlaves) {
            addSlave (slave);
            SlaveInfo info = this.getSlaveInfo (slave);   
            if (info.hasMeanCurvatureNormal ()) {
               //info.enableConformalModesUpdate (false);
               //info.updateCellLaplacian ();
               //info.updateMeanCurvatureNormal ();
            }
            assignSlaveInfo (slave);
         }
      }

      // add map
      map = new SurfaceFixedPoint3dMap
      <VertexToPointCorrespondence> ();
      setCloudMap (map);

      // build allocator
      UniformMeshFeatureSubsampler[] allos = 
      new UniformMeshFeatureSubsampler [Src2TgtMap.size ()];
      int idx = 0;
      for (MeshBase src : Src2TgtMap.keySet ()) {
         MeshBase tgt = Src2TgtMap.get (src);

         UniformMeshFeatureSubsampler allo1 = 
         new UniformMeshFeatureSubsampler ();
         allo1.addMesh (src);
         allo1.setTargetMesh (tgt);
         allo1.setAllocateFeature (MeshFeature.Vertex);
         allo1.setDirection (true);
         allos [idx] = allo1;
         idx++;
      }

      // allocate correspondence
      try {
         allocateCorrespondences (allos);
      }
      catch (ReflectiveOperationException e) {
         System.err.println ("Failed to allocator correspondence");
         e.printStackTrace();
      }

      /*
      try {
         map.buildLocks (AlignmentType.RIGID);
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println ("Failed to build map locks!");
      }*/


      // set iteration actions
      myIAs.addIterativeActions (myAffineIAs);
      myIAs.addIterativeActions (myNFFDIAs);
      setIterativeAction (myIAs);
   }
   
   public ControlPanel createControlPanel (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      myNFFDIAs.controlFFD (ffd, panel, root);
      myIAs.addWidgetsToExternalPanel (panel, root);
      root.addControlPanel (panel);

      return panel;
   }
   
   public ControlPanel createControlPanelForNFFDAction (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      panel.addLabel ("Fixed Point Mapping");

      myNFFDIAs.controlFFD (ffd, panel, root);
      myIAs.setActionIndex (1);
      root.addControlPanel (panel);

      return panel;
   }


   public void addNFFDSlavePanel (SlaveInfo info, 
      ControlPanel panel, RootModel root) {

      Set<TransformerSlaveInfo> slaveInfos = new HashSet<TransformerSlaveInfo> ();
      info.getTransformerInfos (slaveInfos);

      for (TransformerSlaveInfo si : slaveInfos) {
         if ((si instanceof NFFDSlaveInfo)) {
            if (si.getTransformer () == ffd) {
               myNFFDIAs.controlSlave ((NFFDSlaveInfo)si, panel, root);
               return;
            }
         }
      }

      throw new IllegalArgumentException (
      "Not my slave!");
   }

   public NFFDeformer getFFD () {
      return ffd;
   }

   public AffineModelTransformer getAffine () {
      return affine;
   }

   public void upgradeFFD (int times) {
      if (ffd == null) {
         throw new ImproperStateException (
         "FFD not initialized!");
      }
      int iteration = 0;
      while (iteration < times) {
         ffd.upgrade ();
         ffd.defineFFDForSlavesWithoutUpdate ();
         iteration++;
      }
      if (times == 0) {
         ffd.defineFFDForSlavesWithoutUpdate ();
      }
      ffd.updateSlaveInfos ();
      System.out.println ("slave info updated!");
   }

   public void setFFDUpgradeRatio (double ratio) {
      myNFFDIAs.setUpgradeRatio (ratio);
   }
  

   public void setSlaveStrainWeights (FemModel3d fem, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (fem);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setStrainWeight (weights);
   }
   
   public void setSlaveLaplacianWeights (MeshModelAgent mesh, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setLaplacianWeight (weights);
   }
   
   public void setSlaveBendingWeights (MeshModelAgent mesh, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setBendingWeight (weights);
   }
   
   public double getSlaveBendingWeights (MeshModelAgent mesh) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      return ffdSP.getBendingWeight ();
   }

   public void setSlaveMeshQualityWeights (FemModel3d fem, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (fem);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setQualityWeight (weights);
   }
   
   public void setSlaveARAPWeights (MeshModelAgent mesh, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setARAPWeight (weights);
   }
   
   public double getSlaveARAPWeights (MeshModelAgent mesh) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      return ffdSP.getARAPWeight ();
   }
   
   public void setSlaveACAPWeights (MeshModelAgent mesh, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setACAPWeight (weights);
   }
   
   public double getSlaveACAPWeights (MeshModelAgent mesh) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      return ffdSP.getACAPWeight ();
   }
   
   public void setSlaveEdgeWeights (MeshModelAgent mesh, double weights) {
      NFFDSlavePanel ffdSP = getNFFDSlavePanel (mesh);
      if (ffdSP == null) {
         throw new NullPointerException ("FFD slave panel not found!");
      }
      ffdSP.setEdgeWeight (weights);
   }
   
   public void setNFFDEdgeWeight (double weight) {
      NFFDPanel ffdP = myNFFDIAs.getControlPanel (ffd);
      ffdP.setEdgeWeight (weight);
   }
   
   public void setNFFDStrainWeight (double weight) {
      NFFDPanel ffdP = myNFFDIAs.getControlPanel (ffd);
      ffdP.setStrainWeight (weight);
   }

   public NFFDSlavePanel getNFFDSlavePanel (TransformableGeometry slave) {
      SlaveInfo info = ffd.getSlaveInfo (slave);
      if (info == null) {
         throw new NullPointerException (
         "Slave info not found!");
      }
      NFFDPanel ffdP = myNFFDIAs.getControlPanel (ffd);
      if (ffdP == null) {
         throw new ImproperStateException (
         "FFD control panel not initialized!");
      }
      NFFDSlavePanel ffdSP = ffdP.getSlavePanel (info);
      return ffdSP;
   }

   public NFFDTIMIterativeActions getFFDIterativeActions () {
      return myNFFDIAs;
   }

   public AffineTIMIterativeActions getAffineIterativeActions () {
      return myAffineIAs;
   }
   
   public void enableNFFDIterativeActions () {
      myIAs.setActionIndex (1);
      setEnableIteration (true);
   }
   
   public void enableAffineIterativeActions () {
      myIAs.setActionIndex (0);
      setEnableIteration (true);
   }

   public void enableSlaveConformalModesUpdate (boolean enable) {
      for (SlaveInfo info : this.getSlaves ()) {
         info.enableConformalModesUpdate (enable);
      }
   }
}