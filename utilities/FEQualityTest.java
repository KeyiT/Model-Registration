package artisynth.models.swallowingRegistrationTool.utilities;

import java.awt.Color;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemMuscleModel;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.subjectFrank.SubjectModel;
import maspack.render.GL.GLViewer.BlendFactor;

public class FEQualityTest extends RootModel{
   static String myName = "swallow_CT04mm";
   static String dataPath = ArtisynthPath.getSrcRelativePath (
      SubjectModel.class, "subjects");
   static String subjectPath = dataPath + "/" + myName;

   MechModel mechModel;

   public void build (String [] args) {
      FemModel3d fem = readSourceFem ("larynx");
      FEMQualityRenderer FEQRender = new FEMQualityRenderer (fem);
      FEQRender.setColorMap (new double [] {0.1, 0.3, 0.5});
      addController (FEQRender);
      addRenderable (fem);
   }

   static public FemModel3d readFem (String name) {
      FemMuscleModel fem = new FemMuscleModel();
      SubjectModel.readFemWithoutSuffix (fem, 
         subjectPath + "/" + name + "/" + name + "Fem");
      System.out.println (subjectPath + "/" + name + "/" + name + "Fem");

      return fem;
   }
   
   static public FemModel3d readSourceFem (String name) {
      FemMuscleModel fem = new FemMuscleModel();
      SubjectModel.readFemWithoutSuffix (fem, 
         subjectPath + "/registration/" + name + "/" + name + "RepairedFem");
      System.out.println (subjectPath + "/registration/" + name + "/" + name + "RepairedFem");

      return fem;
   }

   public void attach (DriverInterface driver)
   { 
      super.attach (driver);
      getMainViewer().setBackgroundColor(Color.white);

      //getMainViewer ().setEye (new Point3d (0.103649, -0.499269, 0.120798));
      //getMainViewer ().setCenter (new Point3d (0.098447, 0.00317316, 0.0741666));

      getMainViewer ().setBlendSourceFactor (BlendFactor.GL_ONE_MINUS_SRC_ALPHA);
      getMainViewer ().setBlendDestFactor (BlendFactor.GL_SRC_ALPHA);
   }
}
