package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.swallowingRegistrationTool.optimizers.PointMapFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;

public abstract class VertexBasedM2MCorrespondence extends MeshToMeshCorrespondence
implements PointMapFormulator{
   
   @Override
   public MatrixNd makeSourceDataMatrix () {

      int numVer = mySrcMesh.numVertices ();
      MatrixNd pntData = new MatrixNd (numVer, 3);

      int idx = 0;
      Point3d pnt = new Point3d ();
      for (Vertex3d vtx : mySrcMesh.getVertices ()) {
         pnt.set (vtx.getWorldPoint ());
         pntData.setRow (idx++, pnt);
      }

      return pntData;
   }

   @Override
   public MatrixNd makeTargetDataMatrix () {

      int numVer = myTgtMesh.numVertices ();
      MatrixNd pntData = new MatrixNd (numVer, 3);

      int idx = 0;
      Point3d pnt = new Point3d ();
      for (Vertex3d vtx : myTgtMesh.getVertices ()) {
         pnt.set (vtx.getWorldPoint ());
         pntData.setRow (idx++, pnt);
      }

      return pntData;
   }
   
   @Override
   public void setTransformer (RegistrationTransformer tf) {
      // do nothing
   }

   @Override
   public void setCloudMap (CloudMap map) {
      // do nothing
   }

}
