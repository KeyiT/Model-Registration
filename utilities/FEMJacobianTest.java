package artisynth.models.swallowingRegistrationTool.utilities;

import java.io.IOException;

import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.workspace.RootModel;
import artisynth.models.modelOrderReduction.ReadWrite;
import artisynth.models.swallowingRegistrationTool.RegistrationManager;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;

public class FEMJacobianTest extends RootModel{
   
   public void build (String [] args) {
      FemModel3d fem = new FemModel3d ("FEM Test");
      
      MatrixNd rMat = new MatrixNd ();
      try {
         ReadWrite.readMatrix (rMat, RegistrationManager.class, 
            "/data/FEMQualityTest/NodeTestElement.txt");
      }
      catch (IOException e) {
         e.printStackTrace();
      }
      
      // add nodes
      for (int i = 0; i < rMat.rowSize (); i++) {
         Point3d pnt = new Point3d ();
         rMat.getRow (i, pnt);
         FemNode3d node = new FemNode3d (pnt);
         fem.addNode (node);
      }
      
      // add elements
      HexElement ele = new HexElement (
         fem.getNodes ().toArray (
            new FemNode3d [fem.numNodes ()]));
      fem.addElement (ele);
      
      addRenderable (fem);
      
      
   }
}
