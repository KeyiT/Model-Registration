package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;

import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNd;

public abstract class MeshBasedLinearDeformer extends LinearDeformer{
   
   protected ArrayList <PolygonalMesh> myMeshes = 
   new ArrayList<PolygonalMesh> ();

   protected SparseBlockMatrix myK;
   protected VectorNd myTargetF;

   public MeshBasedLinearDeformer () {
      super ();
   }
   
   protected int getKIdx (int meshIdx, int vtxIdx) {
      int idx = 0;
      for (int i = 0; i < meshIdx; i++) {
         PolygonalMesh mesh = myMeshes.get (i);
         idx += mesh.numVertices ();
      }
      idx += vtxIdx;

      return idx;
   }
   
   public SparseBlockMatrix getStiffnessMatrix () {
      return myK;
   }

   public VectorNd getTargetForce () {
      return myTargetF;
   }

   @Override
   public void applyAction () {
      updateMesh();
   }

   @Override
   public void addMasterMesh (MeshBase master) {
      // test mesh
      if (!(master instanceof PolygonalMesh)) {
         throw new IllegalArgumentException (
         "Not polygonal mesh!");
      }
      PolygonalMesh mesh = (PolygonalMesh) master;
      if (!mesh.isManifold ()) {
         throw new IllegalArgumentException (
         "Not Manifold!");
      }
      if (!mesh.isTriangular ()) {
         throw new IllegalArgumentException (
         "Not pure triangular mesh!");
      }
      int numDis = mesh.numDisconnectedVertices ();
      if (numDis > 0) {
         throw new IllegalArgumentException (
            "Have " + numDis + " disconnected vertices!");
      }
      int numDeg = mesh.numDegenerateFaces ();
      if (numDeg > 0) {
         throw new IllegalArgumentException (
            numDeg + "faces existed!");
      }

      int mySize = getSize ();
      setSize (mySize + mesh.numVertices ()*3);

      int idx = mySize;
      for (int i = 0; i < mesh.numVertices (); i++) {
         Point3d pos = new Point3d ();
         mesh.getVertex (i).getWorldPoint (pos);
         for (int j = 0; j < 3; j++) {
            myRestPosition.set (idx, pos.get (j));
            myPosition.set (idx, pos.get (j));
            idx++;
         }
      }

      myMeshes.add (mesh);
   }

   @Override
   /**
    * @param mesh
    */
   public void addMasterMeshes (Collection<MeshBase> masters) {
      for (MeshBase mesh : masters) {
         addMasterMesh (mesh);
      }
   }

   public void getMasterMeshes (List<PolygonalMesh> list) {
      list.addAll (myMeshes);
   }

   public PolygonalMesh getMasterMes (int idx) {
      return myMeshes.get (idx);
   }

   public void clearMeshes () {
      myMeshes.clear ();
      setSize (0);
   }

   @Override
   public void removeMasterMesh (MeshBase master) {
      throw new UnsupportedOperationException("");
   }

   @Override
   public boolean containMasterMesh (MeshBase master) {
      return myMeshes.contains (master);
   }


   public void updateMesh () {
      int numVer = 0;
      for (PolygonalMesh mesh : myMeshes) {
         numVer += mesh.numVertices ();
      }
      if (numVer*3 != getSize ()) {
         throw new ImproperSizeException (
         "size not compatible");
      }

      int idx = 0;
      for (PolygonalMesh mesh : myMeshes) {
         for (Vertex3d vtx : mesh.getVertices ()) {
            Point3d pos = new Point3d ();
            myPosition.getSubVector (idx, pos);
            if (mesh.meshToWorldIsIdentity ()) {
               vtx.setPosition (pos);
            }
            else {
               pos.inverseTransform (mesh.getMeshToWorld ());
               vtx.setPosition (pos);
            }
            idx += 3;
         }
      }
   }

   public void updatePositions () {
      int numVer = 0;
      for (PolygonalMesh mesh : myMeshes) {
         numVer += mesh.numVertices ();
      }
      if (numVer*3 != getSize ()) {
         throw new ImproperSizeException (
         "size not compatible");
      }

      int idx = 0;
      for (PolygonalMesh mesh : myMeshes) {
         for (Vertex3d vtx : mesh.getVertices ()) {
            Point3d pos = new Point3d ();
            vtx.getWorldPoint (pos);
            myPosition.setSubVector (idx, pos);
            idx += 3;
         }
      }
   }

   /**
    * set current mesh as initial state
    */
   public void advanceMeshInitialStates() {
      updatePositions ();
      advanceRestPositions ();
   }

   /**
    * map from vertex point to its undeformed position;
    * @return
    */
   public LinkedHashMap<Point3d, Point3d> creatPointMap () {
      LinkedHashMap <Point3d, Point3d> pointMap = 
      new LinkedHashMap <Point3d, Point3d> ();
      int vtxI = 0;
      for (PolygonalMesh mesh: myMeshes) {
         for (Vertex3d vtx : mesh.getVertices ()) {
            Point3d key = vtx.getPosition ();
            Point3d val = new Point3d ();
            myRestPosition.getSubVector (vtxI*3, val);
            pointMap.put (key, val);
            vtxI++;
         }
      }
      return pointMap;
   }

   public List <Vertex3d> collectVertices () {
      List<Vertex3d> vts = new ArrayList<Vertex3d> ();
      for (PolygonalMesh mesh : myMeshes) {
         for (Vertex3d vtx: mesh.getVertices ()) {
            vts.add (vtx);
         }
      }
      return vts;
   }
   
   @Override
   abstract public SparseBlockMatrix createStiffnessMatrix ();
   
   @Override
   abstract public VectorNd creatTargetForce ();
   
   abstract public SparseBlockMatrix createStiffnessMatrix (boolean normalized);
   
   abstract public VectorNd creatTargetForce (boolean normalized);

}
