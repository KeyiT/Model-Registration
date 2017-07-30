package artisynth.models.swallowingRegistrationTool;

import java.awt.Color;

import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import artisynth.models.subjectFrank.SubjectModel;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.render.RenderProps;
import maspack.render.GL.GLViewer.BlendFactor;

public class TemRender extends RootModel{

   public void build (String [] args) {
      PolygonalMesh mesh1;
      PolygonalMesh mesh2;
      
      mesh1 = SubjectModel.readMeshWithoutSuffix (
         ArtisynthPath.getSrcRelativePath (RegistrationManager.class, "geometry/box2sphereARAP1"));
      
      mesh2 = SubjectModel.readMeshWithoutSuffix (
         ArtisynthPath.getSrcRelativePath (RegistrationManager.class, "geometry/box2sphereACAP1"));
      
      addRenderable (mesh1);
      addRenderable (mesh2);
      
      RenderProps.setFaceColor (mesh1, Color.PINK);
      RenderProps.setFaceColor (mesh2, Color.PINK);
      RenderProps.setDrawEdges (mesh1, true);
      RenderProps.setDrawEdges (mesh2, true);
      
   }
   
   public void attach (DriverInterface driver)
   {
      this.getMainViewer().setBackgroundColor(Color.white);
   }

}
