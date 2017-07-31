package artisynth.models.swallowingRegistrationTool.ICP;

import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import artisynth.core.gui.ControlPanel;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.main.IterativeActionsPanel;
import artisynth.models.swallowingRegistrationTool.main.RegistrationManager;
import artisynth.models.swallowingRegistrationTool.correspondences.ClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.SurfaceClosestPoint3dMap;
import artisynth.models.swallowingRegistrationTool.correspondences.VertexToMeshCorrespondence;
import artisynth.models.swallowingRegistrationTool.infoUtilities.UniformMeshFeatureSubsampler;
import artisynth.models.swallowingRegistrationTool.infoUtilities.MeshInfoAllocator.MeshFeature;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadraticLinearOptimizer;
import artisynth.models.swallowingRegistrationTool.transformers.AffineModelTransformer;
import artisynth.models.swallowingRegistrationTool.transformers.MeshBasedLinearDeformer;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.properties.PropertyList;

public abstract class ICPMLDManager extends RegistrationManager{

   public ICPMLDManager (String name) {
      super (name);
   }

   IterativeActionsPanel myIAs = new IterativeActionsPanel ();
   ICPMLDIterativeActions myMLDIAs;
   AffineICPIterativeAction myAffineIAs = new AffineICPIterativeAction ();

   AffineModelTransformer affine;
   MeshBasedLinearDeformer mld;

   ClosestPoint3dMap<VertexToMeshCorrespondence> map;

   //--------------------------implements properties-----------------------//

   protected boolean srcToTgtEnabled = true;
   protected boolean tgtToSrcEnabled = true;

   private double aw = 0.00001;

   public static PropertyList myProps =
   new PropertyList (ICPMLDManager.class, RegistrationManager.class);

   @Override
   public PropertyList getAllPropertyInfo () {
      return myProps;
   }

   static {
      myProps.add ("srcToTgtMapping", "source to target mapping", true);
      myProps.add ("tgtToSrcMapping", "target to source mapping", true);
      myProps.add ("Rigidity", "weights for rigidity energy term", 1.0);
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

   public void setRigidity (double rigidity) {
      aw = rigidity;
      if (myMLDIAs != null) {
         myMLDIAs.setRigidty (rigidity);
      }
   }

   public double getRigidity () {
      if (myMLDIAs != null) {
         return myMLDIAs.getRigidity ();
      }
      return aw;
   }

   public void initialize (Map<PolygonalMesh, MeshBase> Src2TgtMap) {

      affine = new AffineModelTransformer ();
      mld = initializeDeformer ();
      myMLDIAs = initializeMLDIAs ();

      Set<Entry<PolygonalMesh, MeshBase>> ens = Src2TgtMap.entrySet ();
      Iterator it = ens.iterator ();

      while (it.hasNext ()) {
         Entry<PolygonalMesh, MeshBase> me = (Entry<PolygonalMesh, MeshBase>)it.next ();
         addMeshMatchPair (me.getKey (), me.getValue ());
         affine.addMasterMesh (me.getKey ());
         mld.addMasterMesh (me.getKey ());
      }

      // add action
      addAction (affine, new ARUNSVDOptimizer());
      addAction (mld, new SparseQuadraticLinearOptimizer());

      // add map
      map = new SurfaceClosestPoint3dMap
      <VertexToMeshCorrespondence> ();
      //map  = new ClosestPoint3dMap
      //<VertexToMeshCorrespondence>();
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


      // set iteration actions
      myIAs.addIterativeActions (myAffineIAs);
      myIAs.addIterativeActions (myMLDIAs);
      setIterativeAction (myIAs);
   }
   
   abstract public MeshBasedLinearDeformer initializeDeformer ();
   
   abstract public ICPMLDIterativeActions initializeMLDIAs ();


   public ControlPanel createControlPanel (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");
      panel.addLabel ("Mesh Linear Deformer");
      panel.addWidget ("Initial Rigidity", this, "Rigidity");

      myIAs.addWidgetsToExternalPanel (panel, root);
      createRegistrationErrorProbe (root);
      root.addControlPanel (panel);

      return panel;
   }

   public ControlPanel createControlPanelForMLDAction (RootModel root) {
      ControlPanel panel = createRegistrationControlPanel ();

      panel.addLabel ("Closest Point Map");
      panel.addWidget ("source to target mapping", this, "srcToTgtMapping");
      panel.addWidget ("target to source mapping", this, "tgtToSrcMapping");
      panel.addLabel ("Mesh Linear Deformer");
      panel.addWidget ("Initial Rigidity", this, "Rigidity");

      myIAs.setActionIndex (1);
      createRegistrationErrorProbe (root);
      root.addControlPanel (panel);

      return panel;
   }

}
