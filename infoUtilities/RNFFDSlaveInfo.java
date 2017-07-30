package artisynth.models.swallowingRegistrationTool.infoUtilities;

import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemUtilities;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo.SlaveType;
import artisynth.models.swallowingRegistrationTool.transformers.NFFDPoint3d;
import artisynth.models.swallowingRegistrationTool.transformers.ReducedNFFDeformer;
import artisynth.models.swallowingRegistrationTool.utilities.ElementWarpingUtilities;
import maspack.geometry.MeshBase;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.Vector3d;

public class RNFFDSlaveInfo extends TransformerSlaveInfo<ReducedNFFDeformer>{

// slave point --> NFFDPoint(natural coordinate)
   protected LinkedHashMap<Point3d, NFFDPoint3d> myFFDMap = 
   new LinkedHashMap<Point3d, NFFDPoint3d> ();

   SparseBlockMatrix RTB = new SparseBlockMatrix();
   Map <Object, SparseBlockMatrix> ElementRTBs = new LinkedHashMap <Object, SparseBlockMatrix> ();
   SparseBlockMatrix BTK = new SparseBlockMatrix ();
   Map <Object, SparseBlockMatrix> ElementBTKs = new LinkedHashMap <Object, SparseBlockMatrix> ();


   private boolean saveFFDPoint = false;
   private boolean saveBTK = false;
   private boolean saveElementBTK = false;
   private boolean saveRTB = false;
   private boolean saveElementRTB = false;

   public RNFFDSlaveInfo () {
      // do nothing
   }
   
   public RNFFDSlaveInfo (ReducedNFFDeformer ffd, SlaveInfo info) {
      setTransformer (ffd);
      setSlaveInfo (info);
   }

   public void enableFFDPointSaving (boolean enable) {
      saveFFDPoint = enable;
   }

   public boolean isFFDPointSavingEnabled () {
      return saveFFDPoint;
   }

   /**
    * get point in natural coordinate,
    * FFDPoint3d contain undeformed information
    * of this point3d-agent;
    * @param pnt a point3d-agent, it can be
    * {@link FemNode3d#getPosition} or 
    * {@link Vertex3d#getPosition()}.
    * @return
    */
   public NFFDPoint3d getFFDPoint (Point3d pnt) {
      return myFFDMap.get (pnt);
   }

   public Map<Point3d, NFFDPoint3d> getFFDMap () {
      return myFFDMap;
   }

   public SparseBlockMatrix getBasis (Point3d pnt) {
      SparseBlockMatrix B = myFFDMap.get (pnt).createBasisMatrix ();
      return B;
   }

   public SparseBlockMatrix getBasis () {
      if (myMaster == null) {
         throw new ImproperStateException (
         "NFFD deformer not initilized!");
      }
      int num = myMaster.getCtrlPntsNum ();
      int [] cbs = new int [num];

      if (!saveFFDPoint) {
         throw new UnsupportedOperationException (
         "FFD Point not saved");
      }
      int [] rbs = new int [myFFDMap.values ().size ()];

      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      for (int j = 0; j < cbs.length; j++) {
         cbs[j] = 3;
      }
      SparseBlockMatrix Basis = new SparseBlockMatrix (rbs, cbs);

      Set entry = myFFDMap.entrySet ();
      Iterator it = entry.iterator ();
      int idx = 0;
      while (it.hasNext ()) {
         Map.Entry<Point3d, NFFDPoint3d> me = (Map.Entry<Point3d, NFFDPoint3d>) it.next ();
         SparseBlockMatrix B = me.getValue ().createBasisMatrix ();
         MatrixBlock blk = B.firstBlockInRow (0);
         while (blk != null) {
            Basis.addBlock (idx, blk.getBlockCol (), blk.clone ());
            blk = blk.next ();
         }
         idx++;
      }

      /*
      if (myType == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlave;
         int idx = 0;
         for (FemNode3d node : fem.getNodes ()) {
            SparseBlockMatrix B = myFFDMap.get (node.getPosition ()).createBasisMatrix ();
            MatrixBlock blk = B.firstBlockInRow (0);
            while (blk != null) {
               Basis.addBlock (idx, blk.getBlockCol (), blk.clone ());
               blk = blk.next ();
            }
            idx++;
         }
      }*/

      return Basis;
   }



   public void updateBTK () {
      if (!mySlaveInfo.hasStiffness ()) {
         throw new UnsupportedOperationException (
         "This slave does not have stiffness!");
      }

      if ( isElementBTKSavingEnabled () && 
      (! mySlaveInfo.isElementWarpingEnabled ()) &&
      (! mySlaveInfo.isPointWarpingEnabled())) {
         updateBTKFast ();
         return;
      }

      if (! mySlaveInfo.isStiffnessSavingEnabled ()) {
         throw new UnsupportedOperationException (
         "Stiffness not saved");
      }

      // stiffness
      SparseBlockMatrix K = mySlaveInfo.getStiffnessMatrix ();
      // this is based assumption that
      // stiffness matrix created in  
      // nodes order;
      SparseBlockMatrix B = getBasis ();
      BTK.mulTransposeLeftToBlkMat (B, K);
   }

   private void updateBTKFast () {
      if (!mySlaveInfo.hasStiffness ()) {
         throw new UnsupportedOperationException (
         "This slave does not have stiffness!");
      }
      if (! (mySlaveInfo.hasElement ())) {
         throw new UnsupportedOperationException (
         "Slave does not have elements");
      }
      // this is based assumption that
      // stiffness matrix created in  
      // nodes order;
      SparseBlockMatrix B = getBasis ();
      // make matrix
      int [] rbs = new int [B.numBlockCols ()];
      int [] cbs = new int [B.numBlockRows ()];
      for (int ii = 0; ii < rbs.length; ii++) {
         rbs[ii] = 3;
      }
      for (int ii = 0; ii < cbs.length; ii++) {
         cbs[ii] = 3;
      }
      BTK = new SparseBlockMatrix (rbs, cbs);

      if (mySlaveInfo.getSlaveType() == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlaveInfo.mySlave;
         for (FemElement3d ele : fem.getElements ()) {

            SparseBlockMatrix eleBTK = getElementBTK (ele);

            for (int ii = 0; ii < ele.numNodes (); ii++) {
               FemNode3d node = ele.getNodes ()[ii];
               MatrixBlock blk = eleBTK.firstBlockInCol (ii);

               while (blk != null) {
                  MatrixBlock rblk = BTK.getBlock (blk.getBlockRow (), 
                     node.getSolveIndex ());
                  if (rblk == null) {
                     BTK.addBlock (blk.getBlockRow (), 
                        node.getSolveIndex (), blk.clone ());
                  }
                  else {
                     rblk.add (blk);
                  }
                  blk = blk.down ();
               }
            }
         }
      }
   }

   public SparseBlockMatrix getBTK () {
      return BTK;
   }

   public void getBTK (SparseBlockMatrix btk) {
      btk.set (BTK);
   }

   public void enableBTKSaving (boolean enabled) {
      saveBTK = enabled;
   }

   public boolean isBTKSavingEnabled () {
      return saveBTK;
   }



   public void updateElementBTKs () {
      if (!mySlaveInfo.hasStiffness () ) {
         throw new UnsupportedOperationException (
         "slave does not have stiffness");
      }
      if (!mySlaveInfo.hasElement ()) {
         throw new UnsupportedOperationException (
         "slave does not have elements");
      }

      SparseBlockMatrix B = getBasis ();
      ElementBTKs.clear ();

      if (mySlaveInfo.getSlaveType () == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlaveInfo.mySlave;
         for (FemNode3d node : fem.getNodes ()) {
            node.setRestPosition (mySlaveInfo.getPointUndeformedPosition (
               node.getPosition ()));
         }
         // make stiffness matrix
         LinearMaterial lMat = (LinearMaterial)fem.getMaterial ();
         for (FemElement3d ele : fem.getElements ()) {
            SparseBlockMatrix eleK = createElementStiffnessMatrix (
               ele, lMat, mySlaveInfo.warpElements);
            // -----------------making stiffness matrix end------------------ //
            SparseBlockMatrix BTK = new SparseBlockMatrix ();
            BTK.setVerticallyLinked (true);
            SparseBlockMatrix eleB = getElementBasis (ele, B, this);
            BTK.mulTransposeLeftToBlkMat (eleB, eleK);
            ElementBTKs.put (ele, BTK);
         }
      }
   }

   public SparseBlockMatrix createElementStiffnessMatrix (
      FemElement3d ele, LinearMaterial lMat, boolean warp) {

      FemNode3d [] nodes = ele.getNodes ();

      int [] bs = new int [nodes.length];
      for (int i = 0; i < bs.length; i++) {
         bs[i] = 3;
      }
      SparseBlockMatrix eleK = new SparseBlockMatrix (bs, bs);

      IntegrationPoint3d [] igps = ele.getIntegrationPoints ();
      IntegrationData3d [] idata = ele.getIntegrationData ();
      Vector3d [] GNx = null;

      for (int k = 0; k < igps.length; k++) {
         // ---compute shape gradient--- //
         IntegrationPoint3d igp = igps[k];
         //igp.computeJacobian (ele.getNodes ());
         //igp.computeInverseJacobian ();

         double dv = idata[k].getDetJ0 () * igp.getWeight ();
         GNx = igp.updateShapeGradient (idata[k].getInvJ0 ());

         // ---make stiffness matrix--- // 
         for (int i = 0; i < nodes.length; i++) {
            for (int j = 0; j < nodes.length; j++) {
               if (j >= i) {
                  Matrix3x3Block blk = (Matrix3x3Block)eleK.getBlock (i, j);
                  if (blk == null) {
                     blk = new Matrix3x3Block();
                     eleK.addBlock (i, j, blk);
                  }
                  FemUtilities.addMaterialStiffness (blk, GNx[i], 
                     lMat.getYoungsModulus (), lMat.getPoissonsRatio (), 
                     GNx[j], dv); // TODO: change weight
               }
            }
         }
      }


      for (int i = 0; i < nodes.length; i++) {
         for (int j = 0; j < nodes.length; j++) {
            if (j > i) {
               MatrixBlock blk = eleK.getBlock (i, j);
               if (blk != null) {
                  MatrixBlock blkT = blk.createTranspose ();
                  eleK.addBlock (j, i, blkT);
               }
            }
         }
      }

      if (warp) {
         SparseBlockMatrix RMat = ElementWarpingUtilities.
         assembleWarpingMatrix (mySlaveInfo.getElementRotation (ele), ele);
         SparseBlockMatrix RK = new SparseBlockMatrix ();
         RK.mulToBlkMat (RMat, eleK);
         eleK.mulTransposeRightToBlkMat (RK, RMat);
      }
      return eleK;
   }

   public static SparseBlockMatrix getElementBasis (
      FemElement3d ele, SparseBlockMatrix femB, RNFFDSlaveInfo info) {

      FemNode3d [] nodes = ele.getNodes ();
      int [] rbs = new int [nodes.length];
      int [] cbs = new int [femB.numBlockCols ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      for (int j = 0; j < cbs.length; j++) {
         cbs[j] = 3;
      }
      SparseBlockMatrix eleB = new SparseBlockMatrix (rbs, cbs);

      SparseBlockMatrix B;
      for (int i = 0; i < nodes.length; i++) {
         FemNode3d node = nodes[i];
         //int bi = node.getSolveIndex ();
         //MatrixBlock blk = femB.firstBlockInRow (bi);
         B = info.getBasis (node.getPosition ());
         MatrixBlock blk = B.firstBlockInRow (0);
         while (blk != null) {
            int bj = blk.getBlockCol ();
            eleB.addBlock (i, bj, blk.clone ());
            blk = blk.next ();
         }
      }

      return eleB;
   }

   public SparseBlockMatrix getElementBTK (Object element) {
      return ElementBTKs.get (element);
   }

   public void getElementBTK (SparseBlockMatrix ElementBTK, Object element) {
      ElementBTK.set (ElementBTKs.get (element));
   }

   public void enableElementBTKSaving (boolean enabled) {
      saveElementBTK = enabled;
   }

   public boolean isElementBTKSavingEnabled () {
      return saveElementBTK;
   }

   public Point3d evalFFD (Point3d pnt) {
      if (! myFFDMap.containsKey (pnt)) {
         throw new NullPointerException (
         "Point not belong to this slave!");
      }

      return getFFDPoint(pnt).computeFFD ();
   }

   /**
    * update the NFFD point hash map, this will change undeformed
    * positions of all points.
    */
   public void updateFFDPointHashMap () {
      if (myMaster == null) {
         throw new ImproperStateException("FFD not initialized!");
      }
      if (mySlaveInfo.mySlave == null) {
         throw new ImproperStateException("Slave not initialized!");
      }
      if (!mySlaveInfo.hasPoint ()) {
         throw new ImproperStateException("Slave dose not have points");
      }
      myFFDMap.clear ();
      int idx = 0;
      if (mySlaveInfo.getSlaveType() == SlaveType.MeshBody) {
         MeshComponent body = (MeshComponent) mySlaveInfo.mySlave;
         MeshBase mesh = body.getMesh ();
         for (Vertex3d vtx : mesh.getVertices ()) {
            Point3d save = new Point3d(vtx.getWorldPoint ());
            NFFDPoint3d ffdpnt = new NFFDPoint3d();
            ffdpnt.setUndeformedPosition (save);
            ffdpnt.setFFD (myMaster.getFFD());
            ffdpnt.evalUVT ();
            ffdpnt.setIndex (idx++);
            myFFDMap.put (vtx.getPosition (), ffdpnt);
         }
      }
      else if (mySlaveInfo.getSlaveType () == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d) mySlaveInfo.mySlave;
         for (FemNode3d node : fem.getNodes ()) {
            // ignore bad nodes
            if (node.numAdjacentElements () == 0) {
               continue;
            }
            NFFDPoint3d ffdpnt = new NFFDPoint3d();
            ffdpnt.setUndeformedPosition (node.getPosition ());
            ffdpnt.setFFD (myMaster.getFFD());
            ffdpnt.evalUVT ();
            ffdpnt.setIndex (idx++);
            myFFDMap.put (node.getPosition (), ffdpnt);
         }
      }
      else if (mySlaveInfo.getSlaveType () == SlaveType.RigidBody) {
         RigidBody body = (RigidBody) mySlaveInfo.mySlave;
         MeshBase mesh = body.getMesh ();
         for (Vertex3d vtx : mesh.getVertices ()) {
            Point3d save = new Point3d(vtx.getWorldPoint ());
            NFFDPoint3d ffdpnt = new NFFDPoint3d();
            ffdpnt.setUndeformedPosition (save);
            ffdpnt.setFFD (myMaster.getFFD ());
            ffdpnt.evalUVT ();
            ffdpnt.setIndex (idx++);
            myFFDMap.put (vtx.getPosition (), ffdpnt);
         }
      }
      else if (mySlaveInfo.getSlaveType () == SlaveType.Point) {
         Point pnt = (Point) mySlaveInfo.mySlave;
         Point3d save = new Point3d(pnt.getPosition ());
         NFFDPoint3d ffdpnt = new NFFDPoint3d();
         ffdpnt.setUndeformedPosition (save);
         ffdpnt.setFFD (myMaster.getFFD ());
         ffdpnt.evalUVT ();
         ffdpnt.setIndex (idx++);
         myFFDMap.put (pnt.getPosition (), ffdpnt);
      }
   }



   /**
    * info updating
    */
   public boolean update() {
      if (mySlaveInfo.hasPoint() && saveFFDPoint) {
         updateFFDPointHashMap();
      }
      if (mySlaveInfo.hasStiffness () && mySlaveInfo.hasElement() && saveElementBTK) {
         updateElementBTKs();
      }
      if (mySlaveInfo.hasStiffness () && saveBTK) {
         updateBTK();
      }

      return true;
   }






   public void updateRTB () {
      if (! (mySlaveInfo.hasPoint ())) {
         throw new UnsupportedOperationException (
         "Slave does not have points");
      }
      if (! (mySlaveInfo.hasElement ())) {
         throw new UnsupportedOperationException (
         "Slave does not have elements");
      }

      SparseBlockMatrix R = mySlaveInfo.createPointRotationMatrix ();
      RTB.setVerticallyLinked (true);
      RTB.mulTransposeLeftToBlkMat (R, getBasis());
   }

   public SparseBlockMatrix getRTB () {
      return RTB;
   }

   public void getRTB (SparseBlockMatrix rtb) {
      rtb.set (RTB);
   }

   public void enableRTBSaving (boolean enabled) {
      saveRTB = enabled;
   }

   public boolean isRTBSavingEnabled () {
      return saveRTB;
   }

   public void updateElementRTBs () {
      if (! (mySlaveInfo.hasElement ())) {
         throw new UnsupportedOperationException (
         "Slave does not have elements");
      }

      ElementRTBs.clear ();

      if (mySlaveInfo.myType == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlaveInfo.mySlave;
         SparseBlockMatrix femB = getBasis();
         for (FemElement3d ele : fem.getElements ()) {
            RotationMatrix3d Re = null;

            if (mySlaveInfo.ElementRotations.containsKey (ele)) {
               Re = mySlaveInfo.getElementRotation (ele);
            }
            else {
               Re = new RotationMatrix3d ();
               ElementWarpingUtilities.evalWarpingRotation (Re, ele);
            }

            SparseBlockMatrix R  = ElementWarpingUtilities.
            assembleWarpingMatrix (Re, ele);

            SparseBlockMatrix eleRTB = new SparseBlockMatrix ();
            eleRTB.mulTransposeLeftToBlkMat (R, getElementBasis (ele, femB, this));
            ElementRTBs.put (ele, eleRTB);
         }
      }
   }

   public SparseBlockMatrix getElementRTB (Object ele) {
      if (! ElementRTBs.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find this element!");
      }
      return ElementRTBs.get (ele);
   }

   public void getRTB (SparseBlockMatrix rtb, Object ele) {
      if (! ElementRTBs.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find this element!");
      }
      rtb.set (ElementRTBs.get (ele));
   }

   public void enableElementRTBSaving (boolean enabled) {
      saveElementRTB = enabled;
   }

   public boolean isElementRTBSavingEnabled () {
      return saveElementRTB;
   }

}
