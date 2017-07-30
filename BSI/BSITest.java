package artisynth.models.swallowingRegistrationTool.BSI;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.subjectFrank.SubjectModel;
import artisynth.models.subjectFrank.subjects.ICPFrank;
import artisynth.models.subjectFrank.subjects.SwallowPatient;
import artisynth.models.swallowingRegistrationTool.utilities.FEMQualityUtilities;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.MeshBase;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.Shading;

public class BSITest extends ICPFrank{

   protected List<FemModel3d> femSlaves = new ArrayList<FemModel3d> ();
   protected List<MeshModelAgent> meshSlaves = new ArrayList<MeshModelAgent> ();

   String path1 = "subjects/swallow_CT04mm/registration/external/";
   String path2 = "subjects/swallow_CT04mm/registration/";
   
   FemModel3d face;
   FemModel3d tongue;
   FemModel3d larynx;
   FemModel3d softPalate;
   FemModel3d pharynx;

   public void build (String [] args) throws IOException {
      super.build (args);
      RenderProps.setAlpha (renderables ().get ("airway"), 0.85);
      
      BSIManager bsi = new BSIManager ("BSI Test");

      Map <MeshBase, MeshBase> myMeshes = 
      new LinkedHashMap <MeshBase, MeshBase> ();

      // tongue
      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "tongue/tongueSourceMesh.ply"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path1 + "tongue/tongueResultMesh.ply"));

      // upperLarynx
      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "upperLarynx/upperLarynxSourceMesh.ply"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path1 + "upperLarynx/upperLarynxResultMesh.ply"));

      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "airwayForPharynx/airwayForPharynxSourceMesh1.ply"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path1 + "airwayForPharynx/airwayForPharynxResultMesh.ply"));

      /*
      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path1 + "softPalateForPharynx/softPalateForPharynxSourceMesh.ply"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path1 + "softPalateForPharynx/softPalateForPharynxResultMesh.ply"));*/
      
      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "softPalate/softPalateSourceMesh.ply"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "softPalate/softPalateTargetMesh.ply"));

      /*
      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "jaw/jawSourceMesh.vtk"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "jaw/jawResultMesh.ply"));

      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "maxilla/maxillaSourceMesh.vtk"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "maxilla/maxillaResultMesh.ply")); */


      // upperLarynx
      myMeshes.put (
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "hyoid/hyoidSourceMesh.vtk"), 
         ReadWrite.readMesh (SubjectModel.class, 
            path2 + "hyoid/hyoidResultMesh.ply"));

      // add slave
      tongue = this.addPositiveFem ("tongue");
      softPalate = this.addPositiveFem ("softPalate");
      pharynx = this.addPositiveFem ("pharynx");
      larynx = this.addPositiveFem ("larynx");
      pharynx.getRenderProps ().setFaceColor (Color.PINK);
      pharynx.getRenderProps ().setAlpha (0.52);
      tongue.getRenderProps().setFaceColor    ( new Color(0.75f, 0.50f, 0.42f) );
      larynx.getRenderProps().setFaceColor    ( new Color(0.95f, 0.60f, 0.48f) );
      softPalate.getRenderProps().setFaceColor( new Color(0.80f, 0.55f, 0.45f) );
      
      
      this.addPositiveMesh ("thyroid");
      this.addPositiveMesh ("cricoid");
      this.addPositiveMesh ("epiglottis");

      bsi.initialize (myMeshes, femSlaves, meshSlaves);
      addModel (bsi);
      bsi.renderSourceAndTargetMesh (this, 0.882700, 0.882700, 
         Shading.FLAT, Shading.SMOOTH);
      bsi.createControlPanel (this);
      bsi.createRegistrationErrorProbe (this);
   }

   public FemModel3d addPositiveFem (String name) {
      FemModel3d fem = loadFem (name);
      //RenderProps.setLineColor (fem, Color.GRAY);
      RenderProps.setLineWidth (fem, 0);
      RenderProps.setFaceColor (fem, new Color (0.75f, 0.50f, 0.42f));
      RenderProps.setLineWidth (fem, 0);
      RenderProps.setPointSize (fem, 0);
      fem.setElementWidgetSize (0.98);
      femSlaves.add (fem);
      addRenderable (fem);
      return fem;
   }

   public MeshModelAgent addPositiveMesh (String name) {
      MeshModelAgent agent = loadMesh (name);
      RenderProps.setFaceColor (agent, new Color (238, 232, 170));
      meshSlaves.add (agent);
      addRenderable (agent);
      RenderProps.setFaceStyle (agent, FaceStyle.FRONT_AND_BACK);
      RenderProps.setShading (agent, Shading.SMOOTH);
      return agent;
   }

   public FemModel3d loadFem (String name) {
      FemModel3d fem = new FemModel3d ();

      SubjectModel.readFemWithoutSuffix (fem, 
         ArtisynthPath.getSrcRelativePath (SubjectModel.class, 
            path2 + name + "/"+name+ "SourceFem"));

      fem.setName (name);
      return fem;
   }

   public MeshModelAgent loadMesh (String name) {
      MeshModelAgent finish1 = new MeshModelAgent ();

      finish1.represent ( SubjectModel.readMeshWithoutSuffix (
         ArtisynthPath.getSrcRelativePath (SwallowPatient.class, 
            "swallow_CT04mm/registration/"+name+"/"+name+"SourceMesh")));
      finish1.setName (name);

      return finish1;
   }

   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);

      getMainViewer ().setEye (new Point3d (0.113569, -0.357032, 0.108193));
      getMainViewer ().setCenter (new Point3d (0.106427, 0, 0.0741666));
      getMainViewer ().setTransparencyFaceCulling (false);
      getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      getMainViewer ().setBlendDestFactor (BlendFactor.GL_SRC_ALPHA);
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

}
