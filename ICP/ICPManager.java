package artisynth.models.swallowingRegistrationTool.ICP;

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
import artisynth.models.swallowingRegistrationTool.correspondences.PseudoClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.SurfaceClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.VertexToMeshCorrespondence;
import artisynth.models.swallowingRegistrationTool.infoUtilities.UniformMeshFeatureSubsampler;
import artisynth.models.swallowingRegistrationTool.infoUtilities.MeshInfoAllocator.MeshFeature;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.TransformerSlaveInfo;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadraticLinearOptimizer;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.FFDPointNotFoundException;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDSlavePanel;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDeformer;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshICP.AlignmentType;
import maspack.matrix.ImproperStateException;
import maspack.properties.PropertyList;

public class ICPManager extends RegistrationManager{

   IterativeActionsPanel myIAs = new IterativeActionsPanel ();
   NFFDICPIterativeActions myNFFDIAs = new NFFDICPIterativeActions ();
   AffineICPIterativeAction myAffineIAs = new AffineICPIterativeAction ();

   NFFDeformer ffd;
   AffineModelTransformer affine;

   SurfaceClosestPoint3dMap<VertexToMeshCorrespondence> map;

   //--------------------------implements properties-----------------------//

   protected boolean srcToTgtEnabled = true;
   protected boolean tgtToSrcEnabled = true;

   public static PropertyList myProps =
   new PropertyList (ICPManager.class, RegistrationManager.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.add ("srcToTgtMapping", "source to target mapping", true);
      myProps.add ("tgtToSrcMapping", "target to source mapping", true);
   }

   public void setSrcToTgtMapping(boolean dual) {
      srcToTgtEnabled = dual;
      if (map != null) {
         map.enableSourceToTargetMapping (
            srcToTgtEnabled);
      }
   }

   public boolean getSrcToTgtMapping() {
      return srcToTgtEnabled;
   }

   public void setTgtToSrcMapping(boolean dual) {
      tgtToSrcEnabled = dual;
      if (map != null) {
         map.enableTargetToSourceMapping (
            tgtToSrcEnabled);
      }
   }

   public boolean getTgtToSrcMapping() {
      return tgtToSrcEnabled;
   }


   public ICPManager (String name) {
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
      map = new SurfaceClosestPoint3dMap
      <VertexToMeshCorrespondence> ();
      setCloudMap (map);

      // build allocator
      UniformMeshFeatureSubsampler[] allos = 
      new UniformMeshFeatureSubsampler [2 * Src2TgtMap.size ()];
      int idx = 0;
      for (MeshBase src : Src2TgtMap.keySet ()) {
         MeshBase tgt = Src2TgtMap.get (src);

         UniformMeshFeatureSubsampler allo1 = 
         new UniformMeshFeatureSubsampler ();
         allo1.addMesh (src);
         allo1.setTargetMesh (tgt);
         allo1.setAllocateFeature (MeshFeature.Vertex);
         allo1.setDirection (true);

         UniformMeshFeatureSubsampler allo2 = 
         new UniformMeshFeatureSubsampler ();
         allo2.setMaxSampleNum (src.getVertices ().size ());
         allo2.addMesh (tgt);
         allo2.setTargetMesh (src);
         allo2.setAllocateFeature (MeshFeature.Vertex);
         allo2.setDirection (false);
         allos [idx++] = allo1;
         allos [idx++] = allo2;
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

      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");

      myNFFDIAs.controlFFD (ffd, panel, root);
      myIAs.addWidgetsToExternalPanel (panel, root);
      root.addControlPanel (panel);

      return panel;
   }

   public ControlPanel createControlPanelForNFFDAction (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");

      myNFFDIAs.controlFFD (ffd, panel, root);
      myIAs.setActionIndex (1);
      createRegistrationErrorProbe (root);
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

   public NFFDICPIterativeActions getFFDIterativeActions () {
      return myNFFDIAs;
   }

   public AffineICPIterativeAction getAffineIterativeActions () {
      return myAffineIAs;
   }

   public void enableSlaveConformalModesUpdate (boolean enable) {
      for (SlaveInfo info : this.getSlaves ()) {
         info.enableConformalModesUpdate (enable);
      }
   }
}
