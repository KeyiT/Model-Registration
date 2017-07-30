package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.TransformGeometryContext;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.infoUtilities.*;
import maspack.geometry.MeshBase;
import maspack.geometry.PointMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;
import maspack.render.Renderable;
import maspack.util.ArraySort;

public class NFFDeformer extends BlendLinear3dDeformer
implements TrackableTransformer, HasRenderableModel, SubdividableBL3dDeformer{

   NFFDController myFFD;

   private boolean isTracking = false;
   // if mapping matrix is valid, true otherwise false
   private boolean validP = false;
   // for recording mapping matrix
   private ArrayList <MatrixNd> trackingPList = new ArrayList<MatrixNd> ();
   private ArrayList <Point3d> trackingBBEndVertexList = new ArrayList <Point3d>();
   private ArrayList <RigidTransform3d> trackingBBPoseList = new ArrayList <RigidTransform3d>();
   private ArrayList <int []> trackingNumCtrlPntsList = new ArrayList <int []>();
   private ArrayList <int []> trackingDegreesList = new ArrayList <int []>();


   public NFFDeformer (int [] numCtrlPnts, int [] degrees, 
      RigidTransform3d ctrlGridPose, Point3d BBEndVertex) {
      super();
      enableFastTranform (true);
      myFFD = new NFFDController ();
      myFFD.initializeNFFD (numCtrlPnts, degrees, ctrlGridPose, BBEndVertex);
   }

   /**
    * Initialize this deformer as cubic B-Spine; In each direction the 
    * control number is 4.
    * @param mesh
    */
   public NFFDeformer (MeshBase mesh) {
      super();
      enableFastTranform (true);
      int [] numCtrlPnts  = {4, 4, 4};
      int [] degrees = {3, 3, 3};
      myFFD = new NFFDController ();
      addMasterMesh (mesh);
      myFFD.initializeNFFD (numCtrlPnts, degrees, mesh);
   }

   public NFFDeformer (int numCtrlPnts, int degree, MeshBase mesh) {
      super();
      enableFastTranform (true);
      int [] degrees = {degree, degree, degree};
      int [] NumCtrlPnts = {numCtrlPnts, numCtrlPnts, numCtrlPnts};
      myFFD = new NFFDController ();
      addMasterMesh (mesh);
      myFFD.initializeNFFD (NumCtrlPnts, degrees, mesh);
   }

   public NFFDeformer (Collection<MeshBase> meshes) {
      super();
      enableFastTranform (true);
      int [] numCtrlPnts  = {4, 4, 4};
      int [] degrees = {3, 3, 3};
      myFFD = new NFFDController ();
      addMasterMeshes (meshes);
      myFFD.initializeNFFD (numCtrlPnts, degrees, meshes);
   }

   public NFFDeformer (int numCtrlPnts, int degree, Collection<MeshBase> meshes) {
      super();
      enableFastTranform (true);
      int [] degrees = {degree, degree, degree};
      int [] NumCtrlPnts = {numCtrlPnts, numCtrlPnts, numCtrlPnts};
      myFFD = new NFFDController ();
      addMasterMeshes (meshes);
      myFFD.initializeNFFD (NumCtrlPnts, degrees, meshes);
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
    * @return {@link NFFD3d}
    */
   public NFFD3d getFFD () {
      return myFFD.myFFD;
   }

   /**
    * 
    * @return {@link NFFDController}
    */
   public NFFDController getFFDController () {
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
      return myFFD.myFFD.numControlPoints ();
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
      getControlPoints ();
      
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

      if (isTracking) {
         MatrixNd PMat = new MatrixNd ();
         super.getCoef (PMat);
         if (validP) {
            MatrixNd tP = trackingPList.get (trackingPList.size ()-1);
            tP.set (PMat);
         }
         else {
            int [] numlist = myFFD.myFFD.getCtrlPntsNum ();
            Point3d bbEnd = new Point3d ();
            myFFD.myFFD.getBBEndVertex (bbEnd);
            RigidTransform3d pose = new RigidTransform3d ();
            myFFD.myFFD.getObjToWorld (pose);
            int [] degrees = myFFD.myFFD.getDegrees ();

            trackingBBEndVertexList.add (bbEnd);
            trackingBBPoseList.add (pose);
            trackingDegreesList.add (degrees);
            trackingNumCtrlPntsList.add (numlist);
            trackingPList.add (PMat);
            validP = true;
         }
      }
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
      fastTransformPointBasedSlaves ();
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
   public void startTracking () {
      // check for tracking data
      int np = trackingPList.size ();
      int nv = trackingBBEndVertexList.size ();
      int npo = trackingBBPoseList.size ();
      int nd=  trackingDegreesList.size ();
      int nn = trackingNumCtrlPntsList.size ();
      if (np != nd || np != nv || np != npo || np != nn) {
         throw new ImproperSizeException ("FFD tracking data error!");
      }
      isTracking = true;
   }

   @Override
   public void startNewTracking () {
      isTracking = true;
      trackingPList.clear ();
      trackingBBEndVertexList.clear ();
      trackingNumCtrlPntsList.clear ();
      trackingBBPoseList.clear ();
      trackingDegreesList.clear ();
      validP = false;
   }

   @Override
   public void endTracking () {
      isTracking = false;
   }

   // TODO
   @Override
   public void makeAccumulatedTransform (MeshBase mesh) {

      int np = trackingPList.size ();
      int nv = trackingBBEndVertexList.size ();
      int npo = trackingBBPoseList.size ();
      int nd=  trackingDegreesList.size ();
      int nn = trackingNumCtrlPntsList.size ();
      if (np != nd || np != nv || np != npo || np != nn) {
         throw new ImproperSizeException ("FFD tracking data error!");
      }

      for (int i = 0; i < np; i++) {
         NFFDeformer ffd = new NFFDeformer (
            trackingNumCtrlPntsList.get (i),
            trackingDegreesList.get(i),
            trackingBBPoseList.get (i),
            trackingBBEndVertexList.get (i));
         ffd.endTracking ();
         ffd.setCoef (trackingPList.get (i));
         ffd.getFFD ().updateMesh (mesh);
      }
   }


   public void makeAccumulatedTransform (TransformableGeometry geometry) {

      int np = trackingPList.size ();
      int nv = trackingBBEndVertexList.size ();
      int npo = trackingBBPoseList.size ();
      int nd=  trackingDegreesList.size ();
      int nn = trackingNumCtrlPntsList.size ();
      if (np != nd || np != nv || np != npo || np != nn) {
         throw new ImproperSizeException ("FFD tracking data error!");
      }

      for (int i = 0; i < np; i++) {
         NFFDeformer ffd = new NFFDeformer (
            trackingNumCtrlPntsList.get (i),
            trackingDegreesList.get(i),
            trackingBBPoseList.get (i),
            trackingBBEndVertexList.get (i));
         ffd.endTracking ();
         ffd.setCoef (trackingPList.get (i));
         TransformGeometryContext.transform (geometry, ffd, 0);
      }

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
   public NFFDSlaveInfo createSubSlaveInfo (
      TransformableGeometry slave, SlaveInfo info) {
      NFFDSlaveInfo ffdInfo = new NFFDSlaveInfo ();
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
            NFFDSlaveInfo slaveInfo = (NFFDSlaveInfo) 
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
   
   //TODO
   public boolean hasLargeDistortion () {
      boolean large = false;
      NFFD3d ffd = getFFD ();
      // should not modify them
      double [][] rests = ffd.restCtrlPnts;
      Vector4d [] myPnts = myFFD.myFFD.getControlPoints ();
      
     
      Point3d pnt1 = new Point3d (rests[0]);
      Point3d pnt2 = new Point3d (rests[1]);
      double th = pnt1.distance (pnt2) / 3;
      for (int i = 0; i < myPnts.length; i++) {
         double dis0 = Math.abs (rests[i][0] - myPnts[i].x);
         double dis1 = Math.abs (rests[i][1] - myPnts[i].y);
         double dis2 = Math.abs (rests[i][2] - myPnts[i].z);
         
         if (dis0 >= th || dis1 >= th || dis2 >= th) {
            large = true;
            break;
         }
      }
      
      return large;
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
      validP = false;
      validSubdomainP = false;
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
      validP = false;
      validSubdomainP = false;
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
      validP = false;
      validSubdomainP = false;
   }

   /**
    * regenerate FFD control grid; All slaves and embedded mesh will
    * be inside the new control grid; {@link NFFDSlaveInfo} should be 
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
      validP = false;
      validSubdomainP = false;
   }
   
   /**
    * regenerate FFD control grid; All slaves and embedded mesh will
    * be inside the new control grid; {@link NFFDSlaveInfo} should be 
    * updated after calling this method;
    */
   public void defineFFDForSlavesWithoutUpdate() {
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
      validP = false;
      validSubdomainP = false;
   }

   /**
    * regenerate FFD control grid; The embedded mesh will
    * be inside the new control grid; 
    */
   public void defineFFD () {
      myFFD.myFFD.createControlGrid ();
      myFFD.myFFD.advanceMeshInitialStates ();
      super.updateSlaveInfos ();
      validP = false;
      validSubdomainP = false;
   }
   
   public boolean isPolyHedronConvex () {
      return myFFD.myFFD.isPolyhedralConvex ();
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
         NFFDSlaveInfo slave = (NFFDSlaveInfo) 
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
   

   //------------------------------- sub-domain implementation -----------------------//
   
   private boolean subdivision = false;
   
   private int numSubdomains = 1;
   private int [][] mySubdomains = new int [3][];
   private boolean validSubdomainP = false;
   
   private int subdomainIndex = 0;
   private boolean [] subCPMarks;
   private int numSubCP;

   private int mySubdomainSize = 7;
   
   
   public int makeSubdomains () {
      return makeSubdomains (mySubdomains, mySubdomainSize);
   }
   
   protected int makeSubdomains (
      int [][] subdomains, int subSize) {
      
      int [] numCPs = myFFD.myFFD.getCtrlPntsNum ();
      
      numSubdomains = 1;
      for (int i = 0; i < 3; i++) {
         if (numCPs[i] < subSize) {
            subdomains[i] = new int [1];
            subdomains[i][0] = numCPs[i];
         }
         else {
            int numDom = numCPs [i] / subSize + 1;
            int size = numCPs [i] / numDom;
            numSubdomains *= numDom;
            subdomains[i] = new int [numDom];
            for (int j = 0; j < numDom-1; j++) {
               subdomains [i][j] = size;
            }
            subdomains [i][numDom -1] = 
            numCPs[i] -  size * (numDom -1);
         }
      }
      
      System.out.println ("number of subdomains: " + numSubdomains);
      
      // test
      System.out.println ("Subdomains: ");
      for (int [] dos : subdomains) {
         for (int doi : dos) {
            System.out.print (doi + " ");
         }
         System.out.println ("");
      }
      
      validSubdomainP = true;
      return numSubdomains;
   }
   
   @Override
   public void setWorkingSubdomain (int idx) {
      subdomainIndex = idx;
      
      if (!validSubdomainP) {
         makeSubdomains ();
      }
      
      if (idx >= numSubdomains) {
         throw new IndexOutOfBoundsException ("");
      }
      
      int [] minIdx = new int [3];
      int [] maxIdx = new int [3];
      pickSubdomains (minIdx, maxIdx, 
         mySubdomains, subdomainIndex);
      myFFD.myFFD.createSubdomain (minIdx, maxIdx);
      
      subCPMarks = myFFD.myFFD.getSubdomainCPMarks ();
      numSubCP = myFFD.myFFD.numSubCPs;
      
      // test
      //System.out.println ("\n number of subdomain control points : " + numSubCP);
      
      if (subCPMarks == null) {
         System.out.println ("warning: "
         + "failed to find set working subdomain");
      }
   }
   
   public int getWorkingSubdomain () {
      return subdomainIndex;
   }
   
   public int numSubdomainCtrlPnts () {
      return numSubCP;
   }
   
   public int numSubdomains () {
      return numSubdomains;
   }
   
   public int getWorkingSubdomainIndex () {
      return subdomainIndex;
   }
   
   @Override
   public int [][] findSubdomains (MatrixNd pntsInWorld) {
      
      if (pntsInWorld.colSize () != 3) {
         throw new ImproperSizeException (
            "Incompatible size");
      }
      
      if (!validSubdomainP) {
         makeSubdomains ();
      }
      
      int tmp = subdomainIndex;
      
      int [][] domains = new int [pntsInWorld.rowSize ()][];
      boolean [][] marks = new boolean [domains.length][numSubdomains];
      int [] numDomains = new int [domains.length];
      
      
      for (int idx =  0; idx < numSubdomains; idx++) {
         setWorkingSubdomain (idx);
         for (int i = 0; i < domains.length; i++) {
            Point3d pnt = new Point3d ();
            pntsInWorld.getRow (i, pnt);
            if (myFFD.myFFD.findSubContainingUnit (pnt) != null) {
               marks [i][idx] = true;
               numDomains[i] ++;
            }
         }
      }
      
      setWorkingSubdomain (tmp);
      
      for (int i = 0; i < domains.length; i++) {
         if (numDomains[i] <= 0) {
            return null;
         }
         domains[i] = new int [numDomains [i]];
         int num = 0;
         for (int idx = 0; idx < numSubdomains; idx++) {
            if (marks[i][idx]) {
               domains[i][num++] = idx;
            }
         }
      }

      return domains;
   }

   
   @Override
   public SparseBlockMatrix makeSubdomainBasis (MatrixNd data) {
      if (data.colSize () != 3) {
         throw new ImproperSizeException (
            "Incompatible size");
      }
      
      
      if (!validSubdomainP) {
         setWorkingSubdomain (subdomainIndex);
      }
      
      int [] minIdx = new int [3];
      int [] maxIdx = new int [3];
      pickSubdomains (minIdx, maxIdx, 
         mySubdomains, subdomainIndex);
      
      int [] cbs = new int [myFFD.myFFD.numSubCPs];

      int [] rbs = new int [data.rowSize ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      for (int j = 0; j < cbs.length; j++) {
         cbs[j] = 3;
      }

      SparseBlockMatrix Basis = new SparseBlockMatrix (rbs, cbs);

      Point3d tmpX = new Point3d ();
      for (int i = 0; i < data.rowSize (); i++) {
         tmpX.x = data.get (i, 0);
         tmpX.y = data.get (i, 1);
         tmpX.z = data.get (i, 2);
         SparseBlockMatrix B = makeSubdomainBasis (tmpX);
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

   protected SparseBlockMatrix makeSubdomainBasis (
      Point3d pntInWorld) {
      Point3d uvt = myFFD.myFFD.findPointInSubdomain (
         pntInWorld);
      
      return myFFD.myFFD.createSubBasisMatrix (uvt);
   }

   @Override
   public void getSubdomainCoef (VectorNd Coef) {
      if (subCPMarks == null) {
         throw new UnsupportedOperationException (
            "Subdivision not enabled!");
      }
      Coef.setSize (numSubCP * 3);
      
      Vector4d [] myPnts = myFFD.myFFD.
      getControlPoints ();
      
      int idx = 0;
      int j = 0;
      for (Vector4d pnt: myPnts) {
         if (!subCPMarks[idx++]) {
            continue;
         }
         for (int i = 0; i < 3; i++) {
            Coef.set(j*3 + i, pnt.get (i));
         }
         j++;
      }
   }

   @Override
   public void setSubdomainCoef (VectorNd Coef) {
      if (subCPMarks == null) {
         throw new UnsupportedOperationException (
            "Subdivision not enabled!");
      }
      if (Coef.size () != numSubCP * 3) {
         throw new ImproperSizeException (
            "Incompatible coefficient for the subdomain");
      }
      
      int idx = 0;
      for (int i = 0; i < getCtrlPntsNum (); i++) {
         if (!subCPMarks[i]) {
            
            continue;
         }
         Point3d pnt=  new Point3d ();
         for (int j = 0; j < 3; j++) {
            pnt.set (j, Coef.get (idx*3+j));
         }
         idx ++;
         myFFD.myFFD.setCtrlPntPosition (i, pnt);
      }
   }
  

   private void pickSubdomains (int [] minIdx, int [] maxIdx, 
      int [][] subdomains, int domainIdx) {
      
      int numU  = subdomains[0].length;
      int numV = subdomains[1].length;
      int numT = subdomains[2].length;
      
      int idxU = domainIdx / (numV * numT);
      int idxV = (domainIdx - idxU * numV * numT) / numT;
      int idxT = domainIdx - idxU * numV * numT - idxV * numT;
      
      if (idxU < 0 || idxU >= numU) {
         throw new IndexOutOfBoundsException ("");
      }
      if (idxV < 0 || idxV >= numV) {
         throw new IndexOutOfBoundsException ("");
      }
      if (idxT < 0 || idxT >= numT) {
         throw new IndexOutOfBoundsException ("");
      }
      
      
      minIdx [0] = 0;
      maxIdx [0] = 0;
      for (int i = 0; i <= idxU ; i++) {
         maxIdx [0] += subdomains[0][i];
      }
      minIdx [0] = maxIdx [0] - subdomains[0][idxU];
      maxIdx [0] --;
      
      minIdx [1] = 0;
      maxIdx [1] = 0;
      for (int i = 0; i <= idxV; i++) {
         maxIdx [1] += subdomains[1][i];
      }
      minIdx [1] = maxIdx [1] - subdomains[1][idxV];
      maxIdx [1] --;
      
      minIdx [2] = 0;
      maxIdx [2] = 0;
      for (int i = 0; i <= idxT; i++) {
         maxIdx [2] += subdomains[2][i];
      }
      minIdx [2] = maxIdx [2] - subdomains[2][idxT];
      maxIdx [2] --;
      
      
      // test
      /*
      System.out.println ("\n minIdx:");
      for (int Idx : minIdx) {
         System.out.print (Idx + " ");
      }
      
      System.out.println ("\n maxIdx:");
      for (int Idx : maxIdx) {
         System.out.print (Idx + " ");
      }*/
   }

   @Override
   public void enableSubdivision (boolean enable) {
      subdivision = enable;
   }

   @Override
   public boolean isSubdivisionEnabled () {
      return subdivision;
   }

   
   /**
    * {@inheritDoc}
    *
    */
   @Override
   public void takeOutput (Vector output) {
      if (output == null) {
         return;
      }
      
      if (!subdivision) {
         super.takeOutput (output);
         return;
      }
      
      VectorNd myP = new VectorNd ();
      getSubdomainCoef (myP);
     
      
      if (myP.size () != output.size ()) {
         throw new ImproperSizeException (
         "Incompatible optimization output!");
      }

      VectorNd solution;
      if ( output instanceof VectorNd) {
         solution = (VectorNd) output;
      }
      else {
         solution = new VectorNd ();
         solution.set (output);
      }

      myP.add (solution);
      setSubdomainCoef (myP);
   }




}
