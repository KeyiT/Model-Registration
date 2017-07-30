package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.infoUtilities.RNFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.RNFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import maspack.geometry.MeshBase;
import maspack.geometry.PointMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;
import maspack.render.Renderable;
import maspack.util.ArraySort;

public class ReducedNFFDeformer extends BlendLinear3dDeformer
implements HasRenderableModel{

   ReducedNFFDController myFFD;

   /**
    * Initialize this deformer as linear B-Spine; In each direction the 
    * control number is 2.
    * @param mesh
    */
   public ReducedNFFDeformer () {
      myFFD = new ReducedNFFDController ();
   }
   
   public void buid (Set<MeshBase> meshes, Set<Point3d> slaves) {
      int [] numCtrlPnts  = {2, 2, 2};
      int [] degrees = {1, 1, 1};
      myFFD.initializeNFFD (numCtrlPnts, degrees, meshes, slaves);
   }
   
   public void buid (MeshBase mesh, Set<Point3d> slaves) {
      int [] numCtrlPnts  = {2, 2, 2};
      int [] degrees = {1, 1, 1};
      myFFD.initializeNFFD (numCtrlPnts, degrees, mesh, slaves);
   }
   
   public void build (int [] numCtrlPnts, int [] degrees, 
      MeshBase mesh, Set<Point3d> slaves) {
      myFFD.initializeNFFD (numCtrlPnts, degrees, mesh, slaves);
   }
   
   public void build (int [] numCtrlPnts, int [] degrees, 
      Set<MeshBase> meshes, Set<Point3d> slaves) {
      myFFD.initializeNFFD (numCtrlPnts, degrees, meshes, slaves);
   }

   /**
    * increase the number of control points by 1 in each direction
    * FFD must be defined; 
    * see {@link FFDeformer#defineFFDForSlaves()}; 
    * @param upgradeMarks
    */
   protected void upgradeByThree () {
      int [] nums = myFFD.myFFD.getCtrlPntsNum ();
      myFFD.myFFD.setCtrlPntsNum (nums[0]+1, nums[1]+1, nums[2]+1);
   }

   public void upgrade () {
      Matrix3d stuM = myFFD.myFFD.getUVTMat ();
      int [] nlist = myFFD.myFFD.getCtrlPntsNum ();
      double ratio = 1.25;
      double [] lens =  new double [3];
      double [] unit = new double [3];
      int [] sortIdx = new int [3];
      int maxIdx = -1;
      double max = 0;
      for (int i = 0; i < 3; i++) {
         Vector3d vec = new Vector3d ();
         stuM.getColumn (i, vec);
         lens[i] = vec.norm ();
         unit[i] = lens [i]/nlist[i];
         if (max < unit[i]) {
            max = unit[i];
            maxIdx = i;
         }
         sortIdx[i] = i;
      }

      nlist[maxIdx] += 1;
      unit [maxIdx] = lens[maxIdx] / nlist[maxIdx];
      ArraySort.quickSort (unit, sortIdx);
      changeSeg(unit, sortIdx, lens, nlist);
      myFFD.myFFD.setCtrlPntsNum (nlist);
   }

   private static void changeSeg (double [] unit, int [] sortIdx, double [] lens, int [] nlist) {
      double ratio = 1.25;
      for (int i = 1; i < unit.length; i ++) {
         if (unit[i] > unit[i-1]*ratio) {
            int rn = (int)Math.ceil (lens[sortIdx[i]]/ratio/unit[i-1]);
            nlist[sortIdx[i]] = rn;
         }
      }
   }

   /**
    * 
    * @return {@link ReducedNFFD3d}
    */
   public ReducedNFFD3d getFFD () {
      return myFFD.myFFD;
   }

   /**
    * 
    * @return {@link ReducedNFFDController}
    */
   public ReducedNFFDController getFFDController () {
      return myFFD;
   }

   // ------------------------- slaves ------------------------------// 
   
   private boolean saveFFDPoint = false;
   private boolean saveBTK = false;
   private boolean saveElementBTK = false;
   
   public void enableSlaveElementBTKSaving (boolean enabled) {
      saveElementBTK = enabled;
   }

   public boolean isSlaveElementBTKSavingEnabled () {
      return saveElementBTK;
   }
   
   public void enableSlaveBTKSaving (boolean enabled) {
      saveBTK = enabled;
   }

   public boolean isSlaveBTKSavingEnabled () {
      return saveBTK;
   }
   
   public void enableSlaveFFDPointSaving (boolean enable) {
      saveFFDPoint = enable;
   }

   public boolean isSlaveFFDPointSavingEnabled () {
      return saveFFDPoint;
   }
   

   public void advanceSlaveUndeformedState () {
      if (mySlaves.size () != 0) {
         for (SlaveInfo info : mySlaves.values ()) {
            if (!info.hasPoint ()) continue;
            info.savePointCurretAsUndeformed ();
         }
      }
   }

   // ------------------------- override methods ------------------------------// 

   public int getCtrlPntsNum () {
      return myFFD.myFFD.numReducedCtlPnts ();
   }

   @Override
   public Matrix3d getF (Vector3d r) {
      Matrix3d F = myFFD.myFFD.evalJac (r);
      return F;
   }
   
   @Override
   public void getCoef (VectorNd P) {
      P.setSize (getCtrlPntsNum()*3);
      Vector4d [] myPnts = myFFD.myFFD.
      getReducedControlPoints ();
      
      int idx = 0;
      for (Vector4d pnt: myPnts) {
         for (int i = 0; i < 3; i++) {
            P.set(idx*3 + i, pnt.get (i));
         }
         idx++;
      }
   }

   @Override
   public void setCoef (VectorNd P) {
      myFFD.myFFD.setCtrlPntPositions (P);
   }


   @Override
   public Renderable getTransformerRenderer () {
      return myFFD;
   }

   @Override
   public void applyAction () {
      myFFD.myFFD.updateMeshes ();
   }

   @Override
   public void resetAction () {
      myFFD.myFFD.resetControlGrid ();
      myFFD.myFFD.updateMeshes ();
   }

   /**
    * re-define ffd control grid for slaves and embedded mesh; 
    * update slave info;
    */
   @Override
   public boolean update () {
      // re-define ffd grid
      // and upate master mesh embedded in
      // this FFD cage
      defineFFDForSlaves();
      // update slave info
      return super.update ();
   }

  

   @Override
   public SparseBlockMatrix makeBasis (Point3d X) {
      Point3d uvt = myFFD.myFFD.findPoint (X);
      if (uvt == null) {
         return null;
      }
      return myFFD.myFFD.createBasisMatrix (uvt);
   }

   @Override
   public SparseBlockMatrix makeBasis (MatrixNd Xs) {
      if (Xs.colSize () != 3) {
         throw new ImproperSizeException("Incompatible Size");
      }

      int [] nums = myFFD.myFFD.getCtrlPntsNum ();
      int num = nums[0] * nums[1] * nums[2];
      int [] cbs = new int [num];
      int [] rbs = new int [Xs.rowSize ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      for (int j = 0; j < cbs.length; j++) {
         cbs[j] = 3;
      }

      SparseBlockMatrix Basis = new SparseBlockMatrix (rbs, cbs);

      Point3d tmpX = new Point3d ();
      for (int i = 0; i < Xs.rowSize (); i++) {
         tmpX.x = Xs.get (i, 0);
         tmpX.y = Xs.get (i, 1);
         tmpX.z = Xs.get (i, 2);
         SparseBlockMatrix B = makeBasis (tmpX);
         if (B == null) {
            return null;
         }
         MatrixBlock blk = B.firstBlockInRow (0);

         while (blk != null) {
            Basis.addBlock (i, blk.getBlockCol (), blk.clone ());
            blk = blk.next ();
         }
      }
      return Basis;
   }


   public boolean makeBasis (SparseBlockMatrix Basis, Vector3d X) {
      Point3d uvt = myFFD.myFFD.findPoint (new Point3d (X));
      if (uvt == null) {
         return false;
      }
      return myFFD.myFFD.makeAndAppendBasis (Basis, uvt);
   }

   public boolean makeBasis (SparseBlockMatrix Basis, MatrixNd Xs) {
      if (Xs.colSize () != 3) {
         throw new ImproperSizeException("Incompatible Size");
      }

      Basis.set (new SparseBlockMatrix ());

      Vector3d tmpX = new Vector3d ();
      for (int i = 0; i < Xs.rowSize (); i++) {
         tmpX.x = Xs.get (i, 0);
         tmpX.y = Xs.get (i, 1);
         tmpX.z = Xs.get (i, 2);
         boolean flag = makeBasis (Basis, tmpX);
         if (! flag) {
            return false;
         }
      }
      return true;
   }

   @Override
   public RNFFDSlaveInfo createSubSlaveInfo (
      TransformableGeometry slave, SlaveInfo info) {
      RNFFDSlaveInfo ffdInfo = new RNFFDSlaveInfo ();
      ffdInfo.setSlaveInfo (info);
      ffdInfo.setTransformer (this);
      ffdInfo.enableBTKSaving (saveBTK);
      ffdInfo.enableElementBTKSaving (saveElementBTK);
      ffdInfo.enableFFDPointSaving (saveFFDPoint);
      try {
         ffdInfo.update ();
      }
      catch (Exception e) {
         e.printStackTrace ();
         System.err.println (
            "Failed to update Transformer slave info!");
      }
      return ffdInfo;
   }
   
   /**
    * {@inheritDoc}
    */
   @Override
   public void fastTransformer (
      Set<SlaveInfo> slaves) {
      
      Set<SlaveInfo> done = new HashSet<SlaveInfo>();
      
      Iterator it = slaves.iterator ();
      while (it.hasNext ()) {
         SlaveInfo info = (SlaveInfo)it.next ();
         if (isKnownSlave (info)) {
            RNFFDSlaveInfo slaveInfo = (RNFFDSlaveInfo) 
            info.getTransformerInfo (this);
            if (!info.hasPoint ()) { 
               continue;
            }
            Set<Point3d> keys = slaveInfo.getFFDMap ().keySet ();
            for (Point3d key : keys) {
               info.setPointWorldPosition (key, slaveInfo.evalFFD (key));
            }
            done.add (info);
         }
      }
      
      slaves.removeAll (done);
   }

   // ------------------------- define ffd grid for slaves ------------------------------// 



   public void defineFFDForSlave (MeshBase mesh) {
      PointMesh pmesh = new PointMesh ();
      ArrayList <MeshBase> meshes = new ArrayList<MeshBase> ();
      myFFD.myFFD.getMeshes (meshes);
      for (MeshBase ffdmesh : meshes) {
         for (Vertex3d vtx : ffdmesh.getVertices ()) {
            pmesh.addVertex (new Point3d (vtx.getWorldPoint ()));
         }
      }
      for (Vertex3d vtx : mesh.getVertices ()) {
         pmesh.addVertex (new Point3d (vtx.getWorldPoint ()));
      }
      myFFD.myFFD.createControlGrid (pmesh);
      myFFD.myFFD.advanceMeshInitialStates ();
      super.updateSlaveInfos ();
   }

   public void defineFFDForSlave (FemModel3d fem) {
      PointMesh pmesh = new PointMesh ();
      ArrayList <MeshBase> meshes = new ArrayList<MeshBase> ();
      myFFD.myFFD.getMeshes (meshes);
      for (MeshBase ffdmesh : meshes) {
         for (Vertex3d vtx : ffdmesh.getVertices ()) {
            pmesh.addVertex (new Point3d (vtx.getWorldPoint ()));
         }
      }
      ArrayList <FemNode3d> nodes = new ArrayList<FemNode3d>();
      for(int i = 0; i < fem.getElements ().size (); i++) {
         FemElement3d ele = (FemElement3d)fem.getElement (i);
         for (FemNode3d node : ele.getNodes ()) {
            if (!nodes.contains (node)) {
               nodes.add (node);
               pmesh.addVertex (node.getPosition ());
            }
         }
      }
      myFFD.myFFD.createControlGrid (pmesh);
      myFFD.myFFD.advanceMeshInitialStates ();
      super.updateSlaveInfos ();
   }

   public void defineFFDForSlave (MeshComponent body) {
      MeshBase mesh = body.getMesh ();
      defineFFDForSlave(mesh);
   }

   public void defineFFDForSlave (RigidBody rb) {
      MeshBase mesh = rb.getMesh ();
      defineFFDForSlave(mesh);
   }

   public void defineFFDForSlave (TransformableGeometry slave) {
      if (FemModel3d.class.isAssignableFrom (slave.getClass ())) {
         defineFFDForSlave((FemModel3d)slave);
      }
      else if (MeshComponent.class.isAssignableFrom (slave.getClass ())) {
         defineFFDForSlave((MeshComponent)slave);
      }
      else if (RigidBody.class.isAssignableFrom (slave.getClass ())) {
         defineFFDForSlave((RigidBody)slave);
      } else {
         System.err.println ("Unable to regenerate FFD for " 
         + slave.getClass ().getSimpleName ());
      }
   }

   public void defineFFDForSlave (TransformableGeometry slave, boolean verbose) {
      if (FemModel3d.class.isAssignableFrom (slave.getClass ())) {
         defineFFDForSlave((FemModel3d)slave);
      }
      else if (MeshComponent.class.isAssignableFrom (slave.getClass ())) {
         defineFFDForSlave((MeshComponent)slave);
      }
      else if (RigidBody.class.isAssignableFrom (slave.getClass ())) {
         defineFFDForSlave((RigidBody)slave);
      } else {
         if (verbose) {
            System.err.println ("Unable to regenerate FFD for " 
            + slave.getClass ().getSimpleName ());
         }
      }
   }

   public void defineFFDForSlaves (ArrayList<TransformableGeometry> slaves) {
      PointMesh pmesh = new PointMesh ();
      ArrayList <MeshBase> meshes = new ArrayList<MeshBase> ();
      myFFD.myFFD.getMeshes (meshes);
      for (MeshBase ffdmesh : meshes) {
         for (Vertex3d vtx : ffdmesh.getVertices ()) {
            pmesh.addVertex (new Point3d (vtx.getWorldPoint ()));
         }
      }

      for (TransformableGeometry slave : slaves) {
         if (FemModel3d.class.isAssignableFrom (slave.getClass ())) {
            FemModel3d fem = (FemModel3d) slave;
            ArrayList <FemNode3d> nodes = new ArrayList<FemNode3d>();
            for(int i = 0; i < fem.getElements ().size (); i++) {
               FemElement3d ele = (FemElement3d)fem.getElement (i);
               for (FemNode3d node : ele.getNodes ()) {
                  if (!nodes.contains (node)) {
                     nodes.add (node);
                     pmesh.addVertex (node.getPosition ());
                  }
               }
            }
         }
         else if (MeshComponent.class.isAssignableFrom (slave.getClass ())) {
            MeshComponent meshBody = (MeshComponent)slave;
            MeshBase mesh = meshBody.getMesh ().copy ();
            mesh.transform (mesh.XMeshToWorld);
            for (Vertex3d vtx : mesh.getVertices()) {
               pmesh.addVertex (vtx);
            }
         }
         else if (RigidBody.class.isAssignableFrom (slave.getClass ())) {
            RigidBody rb = (RigidBody)slave;
            MeshBase mesh = rb.getMesh ().copy ();
            mesh.transform (mesh.XMeshToWorld);
            for (Vertex3d vtx : mesh.getVertices()) {
               pmesh.addVertex (vtx);
            }
         }
      }

      myFFD.myFFD.createControlGrid (pmesh);
      myFFD.myFFD.advanceMeshInitialStates ();
      super.updateSlaveInfos ();
   }

   /**
    * regenerate FFD control grid; All slaves and embedded mesh will
    * be inside the new control grid; {@link RNFFDSlaveInfo} should be 
    * updated after calling this method;
    */
   public void defineFFDForSlaves() {
      PointMesh pmesh = new PointMesh ();
      ArrayList <MeshBase> meshes = new ArrayList<MeshBase> ();
      myFFD.myFFD.getMeshes (meshes);
      for (MeshBase ffdmesh : meshes) {
         for (Vertex3d vtx : ffdmesh.getVertices ()) {
            pmesh.addVertex (new Point3d (vtx.getWorldPoint ()));
         }
      }
      for (SlaveInfo info : mySlaves.values ()) {
         Set<Point3d> pnts = info.getPoints ();
         for (Point3d pnt : pnts) {
            pmesh.addVertex ( 
               info.getPointCurrentPosition (pnt) );
         }
      }
      myFFD.myFFD.createControlGrid (pmesh);
      myFFD.myFFD.advanceMeshInitialStates ();
      super.updateSlaveInfos ();
   }

   /**
    * regenerate FFD control grid; The embedded mesh will
    * be inside the new control grid; 
    */
   public void defineFFD () {
      myFFD.myFFD.createControlGrid ();
      myFFD.myFFD.advanceMeshInitialStates ();
      super.updateSlaveInfos ();
   }

   // ------------------------------------fast way to transform slaves---------------------------------- //
   /**
    *  non-generic transform method
    *  fast way to transform slaves
    *  
    *  @return if slave does not support fast transformation,
    *  they will be returned in undeformed hash set.
    */
   public HashSet<TransformableGeometry> fastTransformPointBasedSlaves () {
      HashSet<TransformableGeometry> undeformedSlaves = new 
      HashSet<TransformableGeometry> ();
      for (SlaveInfo info : mySlaves.values ()) {
         RNFFDSlaveInfo slave = (RNFFDSlaveInfo) 
         info.getTransformerInfo (this);
         if (info.hasPoint ()) {
            Set<Point3d> keys = slave.getFFDMap ().keySet ();
            for (Point3d key : keys) {
               info.setPointWorldPosition (key, slave.evalFFD (key));
            }
         }
         else {
            undeformedSlaves.add (info.getSlave ());
         }
      }
      return undeformedSlaves;
   }
}


