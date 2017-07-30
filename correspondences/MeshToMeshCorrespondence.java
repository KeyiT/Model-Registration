package artisynth.models.swallowingRegistrationTool.correspondences;

import maspack.geometry.Face;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;

public abstract class MeshToMeshCorrespondence extends Correspondence{

   protected PolygonalMesh mySrcMesh = new PolygonalMesh ();
   protected PolygonalMesh myTgtMesh = new PolygonalMesh ();

   /**
    * 
    * @param source
    * source should be instance of either <code>Vertex3d</code> or 
    * <code>MeshBase</code> 
    */
   public <T> void addToSourcePool (T source) {
      if ( source instanceof Vertex3d ) {
         mySrcMesh.addVertex ((Vertex3d)source);
      } 
      else if (source instanceof PolygonalMesh) {
         mySrcMesh.addMesh ((PolygonalMesh)source);
      }
      else if (source instanceof MeshBase) {
         for (Vertex3d vtx : ((MeshBase)source).getVertices ()) {
            mySrcMesh.addVertex (vtx);
         }
      }
      else {
         throw new ClassCastException ("Incompatible source!");
      }
   }

   /**
    * 
    * @param target
    * source should be instance of either <code>Vertex3d</code> or 
    * <code>MeshBase</code> 
    */
   public <T> void addToTargetPool (T target) {
      if ( target instanceof Vertex3d ) {
         myTgtMesh.addVertex ((Vertex3d)target);
      } 
      else if (target instanceof PolygonalMesh) {
         myTgtMesh.addMesh ((PolygonalMesh)target);
      }
      else if (target instanceof MeshBase) {
         for (Vertex3d vtx : ((MeshBase)target).getVertices ()) {
            myTgtMesh.addVertex (vtx);
         }
      }
      else {
         throw new ClassCastException ("Incompatible target!");
      }
   }

   /**
    * 
    * @param source
    * source should be instance of either <code>Vertex3d</code> or 
    * <code>MeshBase</code> 
    */
   public <T> void removeSource (T source) {
      if ( source instanceof Vertex3d ) {
         mySrcMesh.removeVertex ((Vertex3d)source);
      } 
      else if (source instanceof PolygonalMesh) {
         for (Face face : ((PolygonalMesh)source).getFaces ()) {
            mySrcMesh.removeFace (face);
         }
         for (Vertex3d vtx : ((PolygonalMesh)source).getVertices ()) {
            mySrcMesh.removeVertex (vtx);
         }
      }
      else if (source instanceof MeshBase) {
         for (Vertex3d vtx : ((MeshBase)source).getVertices ()) {
            mySrcMesh.removeVertex (vtx);
         }
      }
      else {
         throw new ClassCastException ("Incompatible source!");
      }
   }

   /**
    * 
    * @param target
    * source should be instance of either <code>Vertex3d</code> or 
    * <code>MeshBase</code> 
    */
   public <T> void removeTarget (T target) {
      if ( target instanceof Vertex3d ) {
         myTgtMesh.removeVertex ((Vertex3d)target);
      } 
      else if (target instanceof PolygonalMesh) {
         for (Face face : ((PolygonalMesh)target).getFaces ()) {
            myTgtMesh.removeFace (face);
         }
         for (Vertex3d vtx : ((PolygonalMesh)target).getVertices ()) {
            myTgtMesh.removeVertex (vtx);
         }
      }
      else if (target instanceof MeshBase) {
         for (Vertex3d vtx : ((MeshBase)target).getVertices ()) {
            myTgtMesh.removeVertex (vtx);
         }
      }
      else {
         throw new ClassCastException ("Incompatible target!");
      }
   }

   public MeshBase getSourceMesh () {
      return mySrcMesh;
   }

   public MeshBase getTargetMesh () {
      return myTgtMesh;
   }

   public void clearSourcePool () {
      mySrcMesh.clear ();
   }

   public void clearTargetPool () {
      myTgtMesh.clear ();
   }

}
