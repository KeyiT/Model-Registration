package artisynth.models.swallowingRegistrationTool.correspondences;

import artisynth.models.swallowingRegistrationTool.optimizers.PointMapFormulator;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;

public abstract class VertexBasedMeshGlobalMap<C extends VertexBasedM2MCorrespondence> 
extends MeshGlobalMap<C> implements PointMapFormulator{

   @Override
   public MatrixNd makeSourceDataMatrix () {
      
      int numVer = 0;
      for (MeshToMeshCorrespondence info : getInfos ()) {
         numVer += info.getSourceMesh ().numVertices ();
      }
      MatrixNd pntData = new MatrixNd (numVer, 3);
      
      int idx = 0;
      for (MeshToMeshCorrespondence info : getInfos ()) {
         MeshBase src = info.getSourceMesh ();
         Point3d pnt = new Point3d ();
         for (Vertex3d vtx : src.getVertices ()) {
            pnt.set (vtx.getPosition ());
            pntData.setRow (idx++, pnt);
         }
      }
      
      return pntData;
   }

   @Override
   public MatrixNd makeTargetDataMatrix () {
      
      int numVer = 0;
      for (MeshToMeshCorrespondence info : getInfos ()) {
         numVer += info.getTargetMesh ().numVertices ();
      }
      MatrixNd pntData = new MatrixNd (numVer, 3);
      
      int idx = 0;
      for (MeshToMeshCorrespondence info : getInfos ()) {
         MeshBase tgt = info.getTargetMesh ();
         Point3d pnt = new Point3d ();
         for (Vertex3d vtx : tgt.getVertices ()) {
            pnt.set (vtx.getPosition ());
            pntData.setRow (idx++, pnt);
         }
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
