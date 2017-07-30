package artisynth.models.swallowingRegistrationTool;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.subjectFrank.SubjectModel;
import artisynth.models.swallowingRegistrationTool.BSI.BSIManager;
import artisynth.models.swallowingRegistrationTool.utilities.FEMQualityUtilities;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.MeshFactory;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.PointStyle;
import maspack.render.Renderer.Shading;

public class BSIHybridDemo extends RootModel{

   BSIManager bsi = new BSIManager ("Demo");
   protected RenderableComponentList <FemModel3d> fems =
   new RenderableComponentList <FemModel3d> (FemModel3d.class);
   protected RenderableComponentList<MeshModelAgent> meshes = 
   new RenderableComponentList<MeshModelAgent> (MeshModelAgent.class);

   public void build (String [] args) {

      addRenderable (fems);
      addRenderable (meshes);

      // source and target mesh
      Point3d cen1 = new Point3d (0.0, 0.0, 1.0);
      PolygonalMesh box1 = MeshFactory.createBox (2.0, 1.0, 1.0, cen1, 16, 16, 16);
      PolygonalMesh sphere1 = SubjectModel.readMeshWithoutSuffix (
         ArtisynthPath.getSrcRelativePath (BSIDemo.class, "geometry/box2sphere"));

      // slave
      addBeam (2.5, 0.6, 0.75, 10, 4, 5);
      fems.get (0).transformPose (new RigidTransform3d (-1.25, 0, 0.125));
      addBoxSurfaceMesh (1.0, 0.6, 0.6, 4, 4, 4);
      meshes.get (0).transformMesh (new RigidTransform3d (0.5, 0, 0.05));
      addBeam (1.5, 0.6, 0.3, 6, 4, 2);
      fems.get (1).transformPose (new RigidTransform3d (1.85, 0, 0.2));
      addBoxSurfaceMesh (0.75, 0.6, 0.3, 3, 4, 2);
      meshes.get (1).transformMesh (new RigidTransform3d (2.25, 0, -0.15));
      
      // initialize registration
      Map <MeshBase, MeshBase> map =  new HashMap <MeshBase, MeshBase> ();
      map.put (box1, sphere1);
      //map.put (box2, box22);
      //map.put (box3, box33);
      List<FemModel3d> slaves = new ArrayList<FemModel3d> ();
      slaves.addAll (fems);
      List <MeshModelAgent> meshslaves = new ArrayList <MeshModelAgent> ();
      meshslaves.addAll (meshes);
      bsi.initialize (map, slaves, meshslaves);
      addModel (bsi);
      bsi.renderSourceAndTargetMesh (this, 0.45, 0.5, 
         Shading.SMOOTH, Shading.SMOOTH);
      bsi.createControlPanel (this);
      bsi.createRegistrationErrorProbe (this);

      bsi.setEnableIteration (true);
      bsi.setFFDUpgradeRatio (0.08);
      bsi.setNFFDEdgeWeight (0);
      bsi.setNFFDStrainWeight (0.02/3.0);
      bsi.setSlaveStrainWeights (fems.get (0), 0.02);
      bsi.setSlaveStrainWeights (fems.get (1), 0.02);
      bsi.setSlaveACAPWeights (meshes.get (0), 0.02);
      bsi.setSlaveACAPWeights (meshes.get (1), 0.02);
      bsi.upgradeFFD (0);
      bsi.getCloudMap ().update ();
      bsi.enableNFFDIterativeActions ();

      RenderProps srcProp = new RenderProps ();
      srcProp.setFaceColor (Color.RED);
      srcProp.setFaceStyle (FaceStyle.FRONT);
      srcProp.setShading (Shading.SMOOTH);
      //srcProp.setDrawEdges (true);
      //srcProp.setEdgeColor (Color.BLACK);

      RenderProps tgtProp = new RenderProps ();
      tgtProp.setFaceColor (Color.BLUE);
      tgtProp.setFaceStyle (FaceStyle.FRONT);
      tgtProp.setShading (Shading.SMOOTH);
      tgtProp.setEdgeColor (Color.BLACK);
      tgtProp.setAlpha (0.5);

      bsi.setSourceMeshRenderProps (srcProp);
      bsi.setTargetMeshRenderProps (tgtProp);

      // set render properties for fems
      for (FemModel3d fem : fems) {
         fem.setAutoGenerateSurface (true);
         fem.getSurfaceMesh ();
         fem.setSurfaceRendering (SurfaceRender.Shaded);
         
         RenderProps.setFaceColor (fem.getSurfaceMeshComp (), 
            new Color (238, 232, 170));
         RenderProps.setAlpha(fem.getSurfaceMeshComp (), 
            0.5);
      }

      RenderProps.setPointStyle (fems, PointStyle.SPHERE);
      RenderProps.setPointColor (fems, new Color (164, 44, 168));
      RenderProps.setPointRadius (fems, 0.02);
      RenderProps.setLineColor (fems, new Color (164, 41, 246));


      // set render properties for meshes
      RenderProps.setDrawEdges (meshes, true);
      RenderProps.setEdgeColor (meshes, new Color (255, 102, 102));
      RenderProps.setFaceColor (meshes, new Color (238, 232, 170));
      RenderProps.setShading (meshes, Shading.SMOOTH);
   }

   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);

      // w:1000 h:600
      getMainViewer ().setEye (new Point3d (0.299467, -7.89602, 0.146227));
      getMainViewer ().setCenter (new Point3d (0.0625, 0.000499065, 1.09881));
      getMainViewer ().setTransparencyFaceCulling (true);
      getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      getMainViewer ().setBlendDestFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
   }
   
   @Override
   public StepAdjustment advance (
      double t0, double t1, int flags) {
      record ();
      StepAdjustment sa = super.advance (t0, t1, flags);
      for (FemModel3d fem : fems) {
         fem.updateSlavePos ();
      }
      //fems.get (0).updateStressAndStiffness ();
      return sa;
   }
   
   public void addBeam (double wx, double wy, double wz, 
      int numX, int numY, int numZ) {
      FemModel3d beam = new FemModel3d ();
      FemFactory.createHexGrid (beam, wx, wy, wz, numX, numY, numZ);
      fems.add (beam);
   }
   
   public void addBoxSurfaceMesh (double wx, double wy, double wz, 
      int numX, int numY, int numZ) {
      Point3d center = new Point3d ();
      PolygonalMesh box = MeshFactory.createBox (
         wx, wy, wz, center, numX, numY, numZ);
      MeshModelAgent agent = new MeshModelAgent ();
      agent.represent (box);
      meshes.add (agent);
   }
   
   VectorNd triQualityMean = new VectorNd ();
   VectorNd triQualityMin = new VectorNd ();
   VectorNd hexQualityMean = new VectorNd ();
   VectorNd hexQualityMin = new VectorNd ();
   VectorNd meanError = new VectorNd ();
   VectorNd maxError = new VectorNd ();
   public void record () {
      FEMQualityUtilities tool = new FEMQualityUtilities ();
      
      VectorNd qualities = new VectorNd ();
      int num = 0;
      int idx = 0;
      for (MeshModelAgent agent : meshes) {
         num += ((PolygonalMesh)agent.getMesh ()).numFaces ();
      }
      qualities.setSize (num);
      qualities.setZero ();
      for (MeshModelAgent agent : meshes) {
         for (Face face: ((PolygonalMesh)agent.getMesh ()).getFaces ()) {
            qualities.set (idx++, tool.evalMeanRatio (face));
         }
      }
      triQualityMean.setSize (triQualityMean.size () + 1);
      triQualityMin.setSize (triQualityMin.size () + 1);
      triQualityMean.set (triQualityMean.size () - 1, qualities.mean ());
      triQualityMin.set (triQualityMin.size () - 1, qualities.minElement ());
      
      
      qualities = new VectorNd ();
      num = 0;
      idx = 0;
      for (FemModel3d agent : fems) {
         num += agent.numElements ();
      }
      qualities.setSize (num);
      qualities.setZero ();
      for (FemModel3d agent : fems) {
         for (FemElement3d ele: agent.getElements ()) {
            qualities.set (idx++, tool.evalMeanRatio (ele));
         }
      }
      hexQualityMean.setSize (hexQualityMean.size () + 1);
      hexQualityMin.setSize (hexQualityMin.size () + 1);
      hexQualityMean.set (hexQualityMean.size () - 1, qualities.mean ());
      hexQualityMin.set (hexQualityMin.size () - 1, qualities.minElement ());
      
      meanError.setSize (meanError.size () + 1);
      maxError.setSize (maxError.size () + 1);
      meanError.set (meanError.size () - 1, bsi.myErr);
      maxError.set (maxError.size () - 1, bsi.myMaxErr);
   }
   
   public void report (String dirPath) throws IOException {
      double [] vals = new double [triQualityMin.size ()];
      for (int i=  0; i < vals.length; i++) {
         vals[i] = triQualityMin.get (i);
      }
      ReadWrite.writeArrayToFile (vals, dirPath + "minTriQuality.txt");
      
      vals = new double [triQualityMean.size ()];
      for (int i=  0; i < vals.length; i++) {
         vals[i] = triQualityMean.get (i);
      }
      ReadWrite.writeArrayToFile (vals, dirPath + "meanTriQuality.txt");
      
      vals = new double [hexQualityMin.size ()];
      for (int i=  0; i < vals.length; i++) {
         vals[i] = hexQualityMin.get (i);
      }
      ReadWrite.writeArrayToFile (vals, dirPath + "minhexQuality.txt");
      
      vals = new double [hexQualityMean.size ()];
      for (int i=  0; i < vals.length; i++) {
         vals[i] = hexQualityMean.get (i);
      }
      ReadWrite.writeArrayToFile (vals, dirPath + "meanhexQuality.txt");
      
      vals = new double [meanError.size ()];
      for (int i=  0; i < vals.length; i++) {
         vals[i] = meanError.get (i);
      }
      ReadWrite.writeArrayToFile (vals, dirPath + "meanError.txt");
      
      vals = new double [maxError.size ()];
      for (int i=  0; i < vals.length; i++) {
         vals[i] = maxError.get (i);
      }
      ReadWrite.writeArrayToFile (vals, dirPath + "maxError.txt");
   }

}