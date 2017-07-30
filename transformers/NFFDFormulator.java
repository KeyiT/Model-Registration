package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.Map.Entry;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemUtilities;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.correspondences.CloudMap;
import artisynth.models.swallowingRegistrationTool.infoUtilities.CloudInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.EdgeInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.NFFDSlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo.SlaveType;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.optimizers.DenseQuadLSFormulator;
import artisynth.models.swallowingRegistrationTool.optimizers.Formulator;
import artisynth.models.swallowingRegistrationTool.optimizers.LOSparseFormulatorBase;
import artisynth.models.swallowingRegistrationTool.optimizers.SparseQuadLSFormulator;
import artisynth.models.swallowingRegistrationTool.utilities.ElementWarpingUtilities;
import artisynth.models.swallowingRegistrationTool.utilities.FEMQualityUtilities;
import artisynth.models.swallowingRegistrationTool.utilities.GeometryOperator;
import maspack.geometry.Face;
import maspack.geometry.HalfEdge;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.ScaledRigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class NFFDFormulator implements Formulator{

   protected NFFDeformer myFFD;

   public NFFDFormulator () {
   }

   public NFFDFormulator (NFFDeformer ffd) {
      myFFD = ffd;
   }

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      if (tf instanceof NFFDeformer) {
         myFFD = (NFFDeformer)tf;
         return;
      }
      throw new ClassCastException ("Incompatible transformer!");
   }

   public NFFDeformer getTransformer () {
      return myFFD;
   }

   @Override
   public void setCloudMap (CloudMap map) {
      // hook here for override
   }


   /**
    * making virtual nodes stay at where they are
    * @param form
    * @param weight
    * 
    * @return Hessian matrix
    */
   public MatrixNd regularizeGridByDisplace (DenseQuadLSFormulator form, double weight) {
      int [] nums = myFFD.getFFD ().getCtrlPntsNum ();
      int num = nums[0] * nums[1] * nums[2];

      MatrixNd H = new MatrixNd (num*3, num*3);
      H.setIdentity ();

      double norm = num*3;
      H.scale (weight/ norm);

      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
   }

   /**
    * making virtual nodes stay at where they are
    * @param form
    * @param weight
    * 
    * @return Hessian matrix
    */
   public SparseBlockMatrix regularizeGridByDisplace (
      SparseQuadLSFormulator form, double weight) {
      int num = myFFD.getCtrlPntsNum ();

      int [] rbs = new int [num];
      int [] cbs = new int [num];
      for (int i = 0; i < num; i++) {
         cbs [i] = 3;
         rbs [i] = 3;
      }

      SparseBlockMatrix H = new SparseBlockMatrix(rbs, cbs);
      Matrix3x3Block blk = new Matrix3x3Block ();
      blk.setIdentity ();

      for (int i = 0; i < num; i++) {
         H.addBlock (i, i, blk.clone ());
      }

      double norm = num*3;
      H.scale (weight/ norm);

      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
   }



   /**
    * mapping goal: limiting ffd virtual edge length and direction change in each 
    * iteration; make quadratic term directly
    * 
    * @param form formulator, if not null this term will be added into it.
    * @param weight
    * 
    * @return Hessian matrix
    */
   public MatrixNd regularizeGridByEdge (DenseQuadLSFormulator form, double weight) {
      int [] nums = myFFD.getFFD ().getCtrlPntsNum ();
      int num = nums[0] * nums[1] * nums[2];

      MatrixNd H = new MatrixNd (num*3, num*3);
      int [][] edges = myFFD.getFFD ().edgeVtxIdx;

      for (int i = 0; i < edges.length; i++) { 
         for (int k = 0; k < 3; k++) {
            double val = H.get (edges[i][0]*3+k, edges[i][0]*3+k);
            H.set (edges[i][0]*3+k, edges[i][0]*3+k, val+1);
            val = H.get (edges[i][1]*3+k, edges[i][1]*3+k);
            H.set (edges[i][1]*3+k, edges[i][1]*3+k, val+1);
            val = H.get (edges[i][1]*3+k, edges[i][0]*3+k);
            H.set (edges[i][1]*3+k, edges[i][0]*3+k, val-1);
            val = H.get (edges[i][0]*3+k, edges[i][1]*3+k);
            H.set (edges[i][0]*3+k, edges[i][1]*3+k, val-1);
         }
      }

      //double norm = H.frobeniusNorm ();
      H.scale (weight);


      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
   }

   /**
    * mapping goal: limiting ffd virtual edge length and direction change in each 
    * iteration; make quadratic term directly
    * 
    * @param form formulator, if not null this term will be added into it.
    * @param weight
    * 
    * @return Hessian matrix
    */
   public SparseBlockMatrix regularizeGridByEdge (SparseQuadLSFormulator form, double weight) {

      int num = myFFD.getCtrlPntsNum ();
      int [] rbs = new int [num];
      int [] cbs = new int [num];
      for (int i = 0; i < num; i++) {
         cbs [i] = 3;
         rbs [i] = 3;
      }

      SparseBlockMatrix H = new SparseBlockMatrix(rbs, cbs);
      Matrix3x3Block blk;
      int [][] edges = myFFD.getFFD ().edgeVtxIdx;

      for (int i = 0; i < edges.length; i++) { 
         int e0 = edges[i][0];
         int e1 = edges[i][1];

         blk = (Matrix3x3Block)H.getBlock (e0, e0);
         if (blk == null) {
            blk = new Matrix3x3Block ();
            H.addBlock (e0, e0, blk);
         }
         double val;
         for (int k = 0; k < 3; k++) {
            val = blk.get (k, k) + 1;
            blk.set (k, k, val);
         }

         blk = (Matrix3x3Block)H.getBlock (e1, e1);
         if (blk == null) {
            blk = new Matrix3x3Block ();
            H.addBlock (e1, e1, blk);
         }
         for (int k = 0; k < 3; k++) {
            val = blk.get (k, k) + 1;
            blk.set (k, k, val);
         }

         blk = (Matrix3x3Block)H.getBlock (e0, e1);
         if (blk == null) {
            blk = new Matrix3x3Block ();
            H.addBlock (e0, e1, blk);
         }
         for (int k = 0; k < 3; k++) {
            val = blk.get (k, k) - 1;
            blk.set (k, k, val);
         }

         blk = (Matrix3x3Block)H.getBlock (e1, e0);
         if (blk == null) {
            blk = new Matrix3x3Block ();
            H.addBlock (e1, e0, blk);
         }
         for (int k = 0; k < 3; k++) {
            val = blk.get (k, k) - 1;
            blk.set (k, k, val);
         }
      }

      //double norm = H.frobeniusNorm ();
      H.scale (weight);

      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
   }


   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param weight
    */
   public double regularizeGridByStrain (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      double weight) {

      int [] nums = myFFD.getFFD ().getCtrlPntsNum ();

      FemModel3d fem = new FemModel3d ();
      VectorNd rest = new VectorNd (myFFD.getFFD ().numControlPoints ()*3);
      VectorNd current = new VectorNd (myFFD.getFFD ().numControlPoints ()*3);

      for (int i = 0; i < myFFD.getFFD ().numControlPoints (); i++) {
         Point3d pnt = myFFD.getFFD ().getCtrlPointRestPosition (i);
         fem.addNode (new FemNode3d (pnt));
         rest.setSubVector (i*3, pnt);
      }
      myFFD.getCoef (current);

      ComponentListView<FemNode3d> nodes = fem.getNodes();

      int wk = (nums[2]) * (nums[1]);
      int wj = (nums[2]);
      //System.out.println (new Vector3d (nums[0], nums[1], nums[2]));
      boolean flag = true;

      HexElement ee =
      new HexElement(
         nodes.get(0),
         nodes.get(1), 
         nodes.get( wj + 1), 
         nodes.get( wj ),

         nodes.get(wk), 
         nodes.get(wk +  1), 
         nodes.get( wk +  wj + 1), 
         nodes.get( wk +  wj ));
      if (ee.isInvertedAtRest ()) {
         flag = false;
      }

      for (int i = 0; i < nums[2]-1; i++) {
         for (int j = 0; j < nums[1]-1; j++) {
            for (int k = 0; k < nums[0]-1; k++) {
               HexElement e = null;
               if (flag) {
                  e =
                  new HexElement(
                     nodes.get(k * wk + j * wj + i),
                     nodes.get(k * wk + j * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i),

                     nodes.get((k + 1) * wk + j * wj + i), 
                     nodes.get((k + 1) * wk + j * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i));
               }
               else {
                  e =
                  new HexElement(

                     nodes.get((k + 1) * wk + j * wj + i), 
                     nodes.get((k + 1) * wk + j * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i + 1), 
                     nodes.get((k + 1) * wk + (j + 1) * wj + i),

                     nodes.get(k * wk + j * wj + i),
                     nodes.get(k * wk + j * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i + 1), 
                     nodes.get(k * wk + (j + 1) * wj + i));
               }

               e.setParity((i + j + k) % 2 == 0 ? 1 : 0);

               fem.addElement(e);
            }
         }
      }
      fem.invalidateStressAndStiffness ();
      fem.setLinearMaterial (1, 0.1, false);
      fem.resetRestPosition ();

      fem.updateStressAndStiffness ();
      SparseBlockMatrix K = new SparseBlockMatrix (fem.getActiveStiffness ());

      // Hessian
      K.scale (-1.0 * weight);

      // proportional term
      VectorNd dis = new VectorNd ();
      dis.sub (current, rest);
      VectorNd prop = new VectorNd ();
      K.mul (prop, dis);

      //constant term
      double c = prop.dot (dis);


      if (form != null) {
         form.addQuadraticTerm (K);
         form.addPropotionalTerm (prop);
         form.addConstantTerm (c);
      }

      if (H != null) {
         H.set (K);
      }

      if (q != null) {
         q.set (prop);
      }

      return c;
   }


   /**
    * set mapping goal to regularize on slave edges. Before estimate slave 
    * edge changes, a global scaling and rotation is applied to the slave
    * which fits them to the current position
    * 
    * @param opt if not null add this term into this formulator
    * @param rA if not null return transform matrix
    * @param rb if not null return target vector
    * @param slaveInfo
    * @param weight weight for this mapping goal
    */
   public void regularizeSlaveEdge(
      LOSparseFormulatorBase opt, SparseBlockMatrix rA, VectorNd rb, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SparseBlockMatrix B = new SparseBlockMatrix ();
      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!(info.hasPoint ())) { 
         System.out.println ("warning: no point info!");
         return;
      }

      // check if slave has edges
      if (info.hasEdge ()) {
         ScaledRigidTransform3d sT = findConformModes (info);
         CloudInfo<EdgeInfo> ci = info.getEdgeCloudInfo ();
         int num = ci.getInfos ().size ();
         // make optimization goal for each edge
         for (int i = 0; i < num; i++) {
            EdgeInfo eInfo = (EdgeInfo)ci.getInfo (i);
            Point3d pnt1 = eInfo.getHead ();
            Point3d pnt2 = eInfo.getTail ();
            // if true a target displacement term will
            // be added to remove history distortion 
            // accumulation. This term relies on current
            // position and original position

            Point3d current1 = info.getPointCurrentPosition (pnt1);
            Point3d current2 = info.getPointCurrentPosition (pnt2);
            Point3d origin1 = new Point3d (info.getPointUndeformedPosition (pnt1));
            Point3d origin2 = new Point3d (info.getPointUndeformedPosition (pnt2));
            origin1.transform (sT);
            origin2.transform (sT);
            Point3d dis1 = new Point3d();
            Point3d dis2 = new Point3d();
            dis1.sub (current1, origin1);
            dis2.sub (current2, origin2);
            //System.out.println("Test");
            //PrintData.printVector (dis1);
            //PrintData.printVector (dis2);
            dis2.sub (dis1);
            B.set (slaveInfo.getBasis (pnt1));
            B.sub (slaveInfo.getBasis (pnt2));

            B.scale (weight);
            dis2.scale (weight);

            if (opt != null) {
               opt.addExternalGoals (B, dis2);
            }

            if (rA != null) {
               rA.set (B);
            }
            if (rb != null) {
               rb.set (dis2);
            }
         }
      }
      else {
         System.out.println ("Warning: no edge info!");
      }
   }


   private ScaledRigidTransform3d findConformModes (SlaveInfo info) {
      MatrixNd src = new MatrixNd  (info.getPoints ().size (), 3);
      MatrixNd tgt = new MatrixNd (src.rowSize (), src.colSize ());

      Set<Entry<Point3d, Point3d>> en = info.getPointMap ().entrySet ();
      Iterator<Entry<Point3d, Point3d>> it = en.iterator ();
      int idx = 0;
      while (it.hasNext ()) {
         Entry<Point3d, Point3d> me = it.next ();
         src.setRow (idx, me.getValue ());
         tgt.setRow (idx, info.getPointCurrentPosition (me.getKey ()));
         idx++;
      }

      AffineTransform3d affine = new AffineTransform3d ();
      double scale = ARUNSVDOptimizer.fitRigid (affine, src, tgt, false);
      ScaledRigidTransform3d rT = new ScaledRigidTransform3d ();
      rT.R.set (affine.A);
      rT.setScale (scale);
      rT.setTranslation (affine.p);

      return rT;
   }

   /**
    * set mapping goal to regularize on slave edge length. This goal would 
    * penalize on length change. 
    * 
    * @param if not null, opt add this term into this formulator
    * @param rA if not null return transform matrix
    * @param rb if not null return target vector
    * @param slaveInfo 
    * @param weight weight for this mapping goal
    */
   public void regularizeSlaveEdgeLenght(
      LOSparseFormulatorBase opt, SparseBlockMatrix rA, VectorNd rb, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SparseBlockMatrix B = new SparseBlockMatrix ();
      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no stiffness info!");
         return;
      }

      // check if slave has edges
      if (info.hasEdge ()) {
         CloudInfo<EdgeInfo> ci = info.getEdgeCloudInfo ();
         int num = ci.getInfos ().size ();
         // make optimization goal for each edge
         for (int i = 0; i < num; i++) {
            EdgeInfo eInfo = (EdgeInfo)ci.getInfo (i);
            Point3d pnt1 = eInfo.getHead ();
            Point3d pnt2 = eInfo.getTail ();
            Point3d current1 = info.getPointCurrentPosition (pnt1);
            Point3d current2 = info.getPointCurrentPosition (pnt2);
            Point3d origin1 = info.getPointUndeformedPosition (pnt1);
            Point3d origin2 = info.getPointUndeformedPosition (pnt2);
            //
            Vector3d originEdge = new Vector3d ();
            Vector3d currentEdge = new Vector3d ();
            originEdge.sub (origin1, origin2);
            currentEdge.sub (current1, current2);
            // current direction with 
            // undeformed length
            currentEdge.normalize ();
            currentEdge.scale (originEdge.norm ());
            Point3d fakeCurrent2 = new Point3d ();
            fakeCurrent2.sub (current1, currentEdge);
            // set goal
            Point3d dis = new Point3d();
            dis.sub (current2, fakeCurrent2);
            //System.out.println (dis);
            B.set (slaveInfo.getBasis (pnt1));
            B.sub (slaveInfo.getBasis (pnt2));

            B.scale (weight);
            dis.scale (weight);

            if (opt != null) {
               opt.addExternalGoals (B, dis);
            }

            if (rA != null) {
               rA.set (B);
            }
            if (rb != null) {
               rb.set (dis);
            }
         }
      }
      else {
         System.out.println ("Warning: no edge info!");
      }
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public double regularizeSlaveStrain (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return 0;
      }
      if (!info.hasStiffness ()) { 
         System.out.println ("warning: no stiffness info!");
         return 0;
      }
      if (!slaveInfo.isBTKSavingEnabled ()) {
         System.out.println ("warning: no BTK info!");
         return 0;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return 0;
      }

      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix SQua = new SparseBlockMatrix (rbs, rbs);
      VectorNd prop = new VectorNd  ();
      double c = 0;

      if (info.getSlaveType () == SlaveType.Fem) {
         // stiffness
         FemModel3d fem = (FemModel3d)info.getSlave ();

         // basis
         // rest position
         // TODO: need modify for bad nodes
         VectorNd rest = new VectorNd (fem.numNodes ()*3);
         VectorNd current = new VectorNd (fem.numNodes () *3);
         int i = 0;
         //stopwatch.checkpoint ("initilization");
         for (FemNode3d node : fem.getNodes ()) {
            //System.out.println (node.getSolveIndex ());
            Point3d re = new Point3d (
               info.getPointUndeformedPosition (node.getPosition ()));
            rest.setSubVector (i*3, re);
            Point3d cu = 
            info.getPointCurrentPosition (node.getPosition ());
            current.setSubVector (i*3, cu);
            i++;
         }
         VectorNd dis = new VectorNd ();
         dis.sub (current, rest);

         // this is based assumption that
         // stiffness matrix created in  
         // nodes order;
         SparseBlockMatrix B = slaveInfo.getBasis ();
         // make matrix
         SparseBlockMatrix BTK = slaveInfo.getBTK ();

         SQua.mulToBlkMat (BTK, B);
         BTK.mul (prop, dis);

         // constant term
         VectorNd p0 =  new VectorNd ();
         slaveInfo.getTransformer ().getCoef (p0);
         VectorNd tmp = new VectorNd ();
         // p0^T * B^T * K * B * p0
         tmp.mul (SQua, p0);
         c += tmp.dot (p0);
         // - 2 p0^T * B^T * K * s0
         tmp.setSize (p0.size ());
         tmp.mul (BTK, rest);
         c -= 2 * tmp.dot (p0);
         // s0^T * K * s0
         tmp.setSize (rest.size ());
         SparseBlockMatrix K = info.
         getStiffnessMatrix ();
         K.mul (tmp, rest);
         c += tmp.dot (rest);

         SQua.scale (weight);
         prop.scale (weight);
         c *= weight;
      }
      else {
         throw new UnsupportedOperationException (
         "Current version only support FEM slave!");
      }

      if (H != null) {
         H.set (SQua);
      }

      if (q != null) {
         q.set (prop);
      }

      if (form != null) {
         form.addQuadraticTerm (SQua);
         form.addPropotionalTerm (prop); 
         form.addConstantTerm (c);
      }

      return c;
   }



   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public double regularizeSlaveStrain (
      DenseQuadLSFormulator form, MatrixNd H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SparseBlockMatrix SQua = new SparseBlockMatrix ();
      VectorNd prop = new VectorNd ();

      double c = regularizeSlaveStrain (null, SQua, prop, slaveInfo, weight);
      MatrixNd Quad = new MatrixNd ();

      if (H != null) {
         H.set (SQua);
      }
      else {
         Quad.set (SQua);
      }
      if (q != null) {
         q.set (prop);
      }

      if (form != null) {
         if (H != null) {
            form.addQuadraticTerm (H);
         }
         else {
            form.addQuadraticTerm (Quad);
         }
         form.addPropotionalTerm (prop); 
         form.addConstantTerm (c);
      }

      return c;
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public double  regularizeWarpedSlaveStrain (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return 0;
      }
      if (!info.hasStiffness ()) { 
         System.out.println ("warning: no stiffness info!");
         return 0;
      }
      if (!info.isElementWarpingEnabled ()) {
         System.out.println ("warning: no element warping info!");
         return 0;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return 0;
      }

      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);

      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix qua = new SparseBlockMatrix (rbs, rbs);
      double c = 0;

      if (info.getSlaveType () == SlaveType.Fem) {
         //slaveInfo.updateElementBTKs ();
         
         // stiffness
         FemModel3d fem = (FemModel3d)info.getSlave ();

         SparseBlockMatrix B = slaveInfo.getBasis ();

         for (FemElement3d ele : fem.getElements ()) {
            FemNode3d [] nodes = ele.getNodes ();
            VectorNd rest = new VectorNd (ele.numNodes ()*3);
            VectorNd current = new VectorNd (ele.numNodes () *3);
            RotationMatrix3d Rele = info.getElementRotation (ele);
            
            
            // TODO
            int i = 0;
            for (FemNode3d node : nodes) {
               Point3d re = new Point3d();
               re.set (info.
                  getPointUndeformedPosition (node.
                     getPosition ()));
               // warping start
               re.transform (Rele);

               // warping end
               rest.setSubVector (i*3, re);
               Point3d cu = 
               info.getPointCurrentPosition (node.getPosition ());
               current.setSubVector (i*3, cu);
               i++;
            }
           double s = info.getElementScaling (ele);
           rest.scale (s);
            VectorNd dis = new VectorNd ();
            dis.sub (current, rest);

            SparseBlockMatrix BTK = new SparseBlockMatrix (slaveInfo.getElementBTK (ele));
            BTK.scale (1 /s /s);
            // TODO
            SparseBlockMatrix quaEle = new SparseBlockMatrix ();
            SparseBlockMatrix eleB = NFFDSlaveInfo.
            getElementBasis (ele, B, slaveInfo);
            quaEle.mulToBlkMat (BTK, eleB);

            VectorNd pro = new VectorNd  ();
            BTK.mul (pro, dis);

            // constant term
            VectorNd p0 =  new VectorNd ();
            slaveInfo.getTransformer ().getCoef (p0);
            VectorNd tmp = new VectorNd ();
            // p0^T * B^T * K * B * po
            tmp.mul (quaEle, p0);
            c += tmp.dot (p0);
            // - 2 p0^T * B^T * K * s0
            tmp.setSize (p0.size ());
            tmp.mul (BTK, rest);
            c -= 2 * tmp.dot (p0);
            // s0^T * K * s0
            SparseBlockMatrix K = 
            slaveInfo.createElementStiffnessMatrix (ele, 
               (LinearMaterial)fem.getMaterial (), false);
            tmp.setSize (rest.size ());
            K.mul (tmp, rest);
            c += tmp.dot (rest);

            qua.add (quaEle);
            Prop.add (pro);
         }

         qua.scale (weight);
         Prop.scale (weight);
         c *= weight;
      }
      else {
         throw new UnsupportedOperationException (
         "Current version only support FEM slave!");
      }

      if (H != null) {
         H.set (qua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {

         form.addQuadraticTerm (qua);
         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }

      return c;
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public double regularizeWarpedSlaveStrain (
      DenseQuadLSFormulator form, MatrixNd H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SparseBlockMatrix SQua = new SparseBlockMatrix ();
      VectorNd prop = new VectorNd ();

      double c = regularizeWarpedSlaveStrain (null, SQua, prop, slaveInfo, weight);
      MatrixNd Quad = new MatrixNd ();

      if (H != null) {
         H.set (SQua);
      }
      else {
         Quad.set (SQua);
      }
      if (q != null) {
         q.set (prop);
      }

      if (form != null) {
         if (H != null) {
            form.addQuadraticTerm (H);
         }
         else {
            form.addQuadraticTerm (Quad);
         }
         form.addPropotionalTerm (prop); 
         form.addConstantTerm (c);
      }

      return c;
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public void regularizeFEMSlaveMeshQuality1 (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return;
      }
      if (!slaveInfo.isBTKSavingEnabled ()) {
         System.out.println ("warning: no BTK info!");
         return;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return;
      }
      if (info.getSlaveType () != SlaveType.Fem) {
         throw new IllegalArgumentException ("Not FEM slave!");
      }

      MatrixNd Quad = new MatrixNd ();
      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);

      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix SQua = new SparseBlockMatrix (rbs, rbs);
      FemModel3d fem = (FemModel3d)info.getSlave ();

      FEMQualityUtilities FQU = new FEMQualityUtilities ();
      double [] etas = FQU.evalMeanRatios (fem);
      double th = pickTH (etas);

      SparseBlockMatrix B = slaveInfo.getBasis ();
      double c = 0;


      int idx = 0;
      for (FemElement3d ele : fem.getElements ()) {

         // find ideal elements
         Point3d [] restNodes = null;
         double eta = etas[idx++];
         if (eta > 0.1) {
            info.updateElement (ele);
            restNodes = FEMQualityUtilities.findIdealElement (ele);
         }
         else {
            FemElement3d rest;
            try {
               rest = ele.getClass ().newInstance ();
               Point3d [] pnts = info.getRestElement (ele);
               for (int i = 0; i < rest.numNodes (); i++) {
                  rest.getNodes ()[i] = new FemNode3d (pnts[i]);
               }
               restNodes = FEMQualityUtilities.findIdealElement (rest);
            }
            catch (InstantiationException | IllegalAccessException e) {
               System.err.println ("Failed to generate a new element");
               e.printStackTrace();
               System.exit (1);
            }
         }
         // measure weights
         double w;
         if (th < 0.2) {
            w = evalGuassianWeight (eta, th);
         }
         else {
            w = evalGuassianWeight (eta, 0.5);
         }
         double sw = 0.5 * evalGuassianWeight (eta, 0.5);

         FemNode3d [] nodes = ele.getNodes ();
         VectorNd rest = new VectorNd (nodes.length*3);
         VectorNd ideal = new VectorNd (nodes.length*3);
         VectorNd current = new VectorNd (nodes.length*3);
         VectorNd dis = new VectorNd (nodes.length*3);
         VectorNd bia = new VectorNd (nodes.length*3);
         RotationMatrix3d Re = null;
         double Se = 0;

         Re = info.getElementRotation (ele);
         Se = info.getElementScaling (ele);
         //w = w/Math.pow (Se, 2);

         int ii = 0;
         for (FemNode3d node : nodes) {
            Point3d re = new Point3d();
            re.set (info.
               getPointUndeformedPosition (node.
                  getPosition ()));

            Point3d id = restNodes[ii];
            ideal.setSubVector (ii*3, id);

            // warping start
            re.transform (Re);
            re.scale (Se);
            // warping end

            rest.setSubVector (ii*3, re);
            Point3d cu = 
            info.getPointCurrentPosition (node.getPosition ());
            current.setSubVector (ii*3, cu);
            ii++;
         }
         dis.sub (current, rest);
         bia.sub (current, ideal);

         SparseBlockMatrix BTK = new SparseBlockMatrix(slaveInfo.getElementBTK (ele));
         BTK.scale (1/Se/Se);
         SparseBlockMatrix quaEle = new SparseBlockMatrix ();
         SparseBlockMatrix eleB = NFFDSlaveInfo.
         getElementBasis (ele, B, slaveInfo);
         quaEle.mulToBlkMat (BTK, eleB);

         VectorNd pro = new VectorNd  ();
         BTK.mul (pro, dis);
         pro.scale (sw);
         Prop.add (pro);
         BTK.mul (pro, bia);
         pro.scale (w);
         Prop.add (pro);

         // constant term
         VectorNd p0 =  new VectorNd ();
         slaveInfo.getTransformer ().getCoef (p0);
         VectorNd tmp = new VectorNd ();
         // p0^T * B^T * K * B * po
         tmp.mul (quaEle, p0);
         c += tmp.dot (p0) * (sw + w);
         // - 2 p0^T * B^T * K * s0
         tmp.setSize (rest.size ());
         BTK.mulTranspose (tmp, p0);
         c -= 2 * tmp.dot (rest) * sw;
         c -= 2 * tmp.dot (ideal) * w;
         // s0^T * K * s0
         SparseBlockMatrix K = 
         slaveInfo.createElementStiffnessMatrix (ele, 
            (LinearMaterial)fem.getMaterial (), false);
         tmp.setSize (rest.size ());
         K.mul (tmp, rest);
         c += tmp.dot (rest) * sw;
         K.mul (tmp, ideal);
         c += tmp.dot (ideal) * w;

         quaEle.scale (sw + w);
         SQua.add (quaEle);
      }

      SQua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (SQua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {

         form.addQuadraticTerm (SQua);

         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public void regularizeFEMSlaveMeshQuality (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return;
      }
      if (!slaveInfo.isBTKSavingEnabled ()) {
         System.out.println ("warning: no BTK info!");
         return;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return;
      }
      if (info.getSlaveType () != SlaveType.Fem) {
         throw new IllegalArgumentException ("Not FEM slave!");
      }

      MatrixNd Quad = new MatrixNd ();
      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);

      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix SQua = new SparseBlockMatrix (rbs, rbs);
      FemModel3d fem = (FemModel3d)info.getSlave ();

      FEMQualityUtilities FQU = new FEMQualityUtilities ();
      double [] etas = FQU.evalMeanRatios (fem);
      double th = pickTH (etas);

      SparseBlockMatrix B = slaveInfo.getBasis ();
      double c = 0;


      int idx = 0;
      for (FemElement3d ele : fem.getElements ()) {

         // find ideal elements
         Point3d [] restNodes = null;
         double eta = etas[idx++];
         //if (eta > 0.1) {
            info.updateElement (ele);
            restNodes = FEMQualityUtilities.findIdealElement (ele);
            FemElement3d restEle = null;
            double volIdeal;
            try {
               restEle = ele.getClass ().newInstance ();
               for (int i = 0; i < restEle.numNodes (); i++) {
                  restEle.getNodes ()[i] = new FemNode3d (restNodes[i]);
               }
               volIdeal = restEle.computeVolumes ();
            }
            catch (InstantiationException | IllegalAccessException e) {
               e.printStackTrace();
               volIdeal = ele.computeVolumes ();
            }
            
         //}
         /*
         else {
            FemElement3d rest;
            try {
               rest = ele.getClass ().newInstance ();
               Point3d [] pnts = info.getRestElement (ele);
               for (int i = 0; i < rest.numNodes (); i++) {
                  rest.getNodes ()[i] = new FemNode3d (pnts[i]);
               }
               restNodes = FEMQualityUtilities.findIdealElement (rest);
            }
            catch (InstantiationException | IllegalAccessException e) {
               System.err.println ("Failed to generate a new element");
               e.printStackTrace();
               System.exit (1);
            }
         }*/
         
         // measure weights
         double w;
         if (th < 0.2) {
            w = evalGuassianWeight (eta, th);
         }
         else {
            w = evalGuassianWeight (eta, 0.5);
         }
         double sw = 0.5 * evalGuassianWeight (eta, 0.5);
         //double sw = 0;
         
         FemNode3d [] nodes = ele.getNodes ();
         VectorNd rest = new VectorNd (nodes.length*3);
         VectorNd ideal = new VectorNd (nodes.length*3);
         VectorNd current = new VectorNd (nodes.length*3);
         VectorNd dis = new VectorNd (nodes.length*3);
         VectorNd bia = new VectorNd (nodes.length*3);
         RotationMatrix3d Re = null;
         double Se = 0;

         Re = info.getElementRotation (ele);
         Se = info.getElementScaling (ele);
         //w = w/Math.pow (Se, 2);

         int ii = 0;
         for (FemNode3d node : nodes) {
            Point3d re = new Point3d();
            re.set (info.
               getPointUndeformedPosition (node.
                  getPosition ()));

            Point3d id = restNodes[ii];
            ideal.setSubVector (ii*3, id);

            // warping start
            re.transform (Re);
            re.scale (Se);
            // warping end

            rest.setSubVector (ii*3, re);
            Point3d cu = 
            info.getPointCurrentPosition (node.getPosition ());
            current.setSubVector (ii*3, cu);
            ii++;
         }
         dis.sub (current, rest);
         bia.sub (current, ideal);
         
         SparseBlockMatrix NiNj = computeElementNiNj (ele);
         VectorNd fe = new VectorNd (ele.numNodes () *3);
         
         //NiNj.mul (fe, bia);
         //fe.scale (1.0/ volIdeal);
         bia.scale (1.0/volIdeal);
         
         SparseBlockMatrix BTK = new SparseBlockMatrix(slaveInfo.getElementBTK (ele));
         BTK.scale (1/Se/Se*sw);
         SparseBlockMatrix quaEle = new SparseBlockMatrix ();
         SparseBlockMatrix eleB = NFFDSlaveInfo.
         getElementBasis (ele, B, slaveInfo);
         quaEle.mulToBlkMat (BTK, eleB);

         VectorNd pro = new VectorNd  ();
         BTK.mul (pro, dis);
         pro.scale (sw);
         Prop.add (pro);
         eleB.mulTranspose (pro, bia);
         pro.scale (w+sw);
         Prop.add (pro);

         // constant term
         VectorNd p0 =  new VectorNd ();
         slaveInfo.getTransformer ().getCoef (p0);
         VectorNd tmp = new VectorNd ();
         // p0^T * B^T * K * B * po
         tmp.mul (quaEle, p0);
         c += tmp.dot (p0) * (sw + w);
         // - 2 p0^T * B^T * K * s0
         tmp.setSize (rest.size ());
         BTK.mulTranspose (tmp, p0);
         c -= 2 * tmp.dot (rest) * sw;
         c -= 2 * tmp.dot (ideal) * w;
         // s0^T * K * s0
         SparseBlockMatrix K = 
         slaveInfo.createElementStiffnessMatrix (ele, 
            (LinearMaterial)fem.getMaterial (), false);
         tmp.setSize (rest.size ());
         K.mul (tmp, rest);
         c += tmp.dot (rest) * sw;
         K.mul (tmp, ideal);
         c += tmp.dot (ideal) * w;

         quaEle.scale (sw + w);
         SQua.add (quaEle);
      }

      SQua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (SQua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {

         form.addQuadraticTerm (SQua);

         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }
   
   
   public static SparseBlockMatrix computeElementNiNj (FemElement3d ele) {
      
      FemNode3d [] nodes = ele.getNodes ();
      IntegrationPoint3d [] igps = ele.getIntegrationPoints ();
      IntegrationData3d [] idata = ele.getIntegrationData ();
      VectorNd Ni = null;
      
      int [] rbs = new int [nodes.length];
      for (int i = 0; i < rbs.length; i++) {
         rbs [i] = 3;
      }
      SparseBlockMatrix eleK = new SparseBlockMatrix (rbs, rbs);
      

      for (int k = 0; k < igps.length; k++) {
         // ---compute shape gradient--- //
         IntegrationPoint3d igp = igps[k];
         //igp.computeJacobian (ele.getNodes ());
         //igp.computeInverseJacobian ();

         // integration weight
         double dv = idata[k].getDetJ0 () * igp.getWeight ();
         // shape function values for this quadrature point
         Ni = igp.getShapeWeights ();

         // ---make NiNj matrix--- // 
         for (int j = 0; j < nodes.length; j++) {
            for (int i = 0; i < nodes.length; i++) {
               if (i >= j) {
                  Matrix3x3Block blk = (Matrix3x3Block)eleK.getBlock (j, i);
                  if (blk == null) {
                     blk = new Matrix3x3Block();
                     eleK.addBlock (j, i, blk);
                  }
                  double NiNj = Ni.get (i) * Ni.get (j);
                  NiNj *= dv;
                  double val = NiNj + blk.get (0, 0);
                  blk.setDiagonal (val, val, val);
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
      
      return eleK;
   }
   
   public static double[] computeElementNi (FemElement3d ele) {
      
      FemNode3d [] nodes = ele.getNodes ();
      IntegrationPoint3d [] igps = ele.getIntegrationPoints ();
      IntegrationData3d [] idata = ele.getIntegrationData ();
      VectorNd Ni = null;
      
      double [] intN = new double [nodes.length];
      
      for (int k = 0; k < igps.length; k++) {
         IntegrationPoint3d igp = igps[k];

         // integration weight
         double dv = idata[k].getDetJ0 () * igp.getWeight ();
         // shape function values for this quadrature point
         Ni = igp.getShapeWeights ();

         // ---make Ni Vector--- // 
         for (int i = 0; i< nodes.length; i++) {
            double ni = Ni.get (i);
            ni *= dv;
            intN[i] += ni;
         }
      }
      
      return intN;
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public void regularizeFEMSlaveMeshQuality1 (
      DenseQuadLSFormulator form, MatrixNd H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return;
      }
      if (!slaveInfo.isBTKSavingEnabled ()) {
         System.out.println ("warning: no BTK info!");
         return;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return;
      }
      if (info.getSlaveType () != SlaveType.Fem) {
         throw new IllegalArgumentException ("Not FEM slave!");
      }

      MatrixNd Quad = new MatrixNd ();
      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);

      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix SQua = new SparseBlockMatrix (rbs, rbs);
      FemModel3d fem = (FemModel3d)info.getSlave ();

      FEMQualityUtilities FQU = new FEMQualityUtilities ();
      double [] etas = FQU.evalMeanRatios (fem);
      double th = pickTH (etas);

      SparseBlockMatrix B = slaveInfo.getBasis ();
      double c = 0;


      int idx = 0;
      for (FemElement3d ele : fem.getElements ()) {

         // find ideal elements
         Point3d [] restNodes = null;
         double eta = etas[idx++];
         if (eta > 0.1) {
            info.updateElement (ele);
            restNodes = FEMQualityUtilities.findIdealElement (ele);
         }
         else {
            FemElement3d rest;
            try {
               rest = ele.getClass ().newInstance ();
               Point3d [] pnts = info.getRestElement (ele);
               for (int i = 0; i < rest.numNodes (); i++) {
                  rest.getNodes ()[i] = new FemNode3d (pnts[i]);
               }
               restNodes = FEMQualityUtilities.findIdealElement (rest);
            }
            catch (InstantiationException | IllegalAccessException e) {
               System.err.println ("Failed to generate a new element");
               e.printStackTrace();
               System.exit (1);
            }
         }
         // measure weights
         double w;
         if (th < 0.2) {
            w = evalGuassianWeight (eta, th);
         }
         else {
            w = evalGuassianWeight (eta, 0.5);
         }
         double sw = 0.5 * evalGuassianWeight (eta, 0.5);

         FemNode3d [] nodes = ele.getNodes ();
         VectorNd rest = new VectorNd (nodes.length*3);
         VectorNd ideal = new VectorNd (nodes.length*3);
         VectorNd current = new VectorNd (nodes.length*3);
         VectorNd dis = new VectorNd (nodes.length*3);
         VectorNd bia = new VectorNd (nodes.length*3);
         RotationMatrix3d Re = null;
         double Se = 0;

         Re = info.getElementRotation (ele);
         Se = info.getElementScaling (ele);
         //w = w/Math.pow (Se, 2);

         int ii = 0;
         for (FemNode3d node : nodes) {
            Point3d re = new Point3d();
            re.set (info.
               getPointUndeformedPosition (node.
                  getPosition ()));

            Point3d id = restNodes[ii];
            ideal.setSubVector (ii*3, id);

            // warping start
            re.transform (Re);
            re.scale (Se);
            // warping end

            rest.setSubVector (ii*3, re);
            Point3d cu = 
            info.getPointCurrentPosition (node.getPosition ());
            current.setSubVector (ii*3, cu);
            ii++;
         }
         dis.sub (current, rest);
         bia.sub (current, ideal);

         SparseBlockMatrix BTK = new SparseBlockMatrix(slaveInfo.getElementBTK (ele));
         BTK.scale (1.0/Se/Se);
         SparseBlockMatrix quaEle = new SparseBlockMatrix ();
         SparseBlockMatrix eleB = NFFDSlaveInfo.
         getElementBasis (ele, B, slaveInfo);
         quaEle.mulToBlkMat (BTK, eleB);

         VectorNd pro = new VectorNd  ();
         BTK.mul (pro, dis);
         pro.scale (sw);
         Prop.add (pro);
         BTK.mul (pro, bia);
         pro.scale (w);
         Prop.add (pro);

         // constant term
         VectorNd p0 =  new VectorNd ();
         slaveInfo.getTransformer ().getCoef (p0);
         VectorNd tmp = new VectorNd ();
         // p0^T * B^T * K * B * po
         tmp.mul (quaEle, p0);
         c += tmp.dot (p0) * (sw + w);
         // - 2 p0^T * B^T * K * s0
         tmp.setSize (rest.size ());
         BTK.mulTranspose (tmp, p0);
         c -= 2 * tmp.dot (rest) * sw;
         c -= 2 * tmp.dot (ideal) * w;
         // s0^T * K * s0
         SparseBlockMatrix K = 
         slaveInfo.createElementStiffnessMatrix (ele, 
            (LinearMaterial)fem.getMaterial (), true);
         tmp.setSize (rest.size ());
         K.mul (tmp, rest);
         c += tmp.dot (rest) * sw;
         K.mul (tmp, ideal);
         c += tmp.dot (ideal) * w;

         quaEle.scale (sw + w);
         SQua.add (quaEle);
      }

      SQua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (SQua);
      }
      else {
         Quad.set (SQua);
      }
      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {
         if (H != null) {
            form.addQuadraticTerm (H);
         }
         else {
            form.addQuadraticTerm (Quad);
         }
         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }

   private static SparseBlockMatrix getElementBasis (
      FemElement3d ele, SparseBlockMatrix femB, NFFDSlaveInfo info) {

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

   private double evalGuassianWeight (double x, double th) {
      double sig = th/2.0;
      if (th == 0) {
         if (x == 0) {
            return 1.0;
         }
         else {
            return 0;
         }
      }
      double w = Math.exp ( - (x * x) / 2.0 / (sig * sig) );
      return w;
   }

   private double pickTH (double [] etas) {
      int num = etas.length;
      if (num == 1) {
         return etas[0];
      }
      else if (num < 1) {
         return 0;
      }
      int thNum = (int)Math.ceil (0.1 * num);
      thNum --;
      double [] tmps = etas.clone ();
      Arrays.sort (tmps);
      
      
      double th = tmps[thNum];
      if (th == 0) {
         return 0;
      }
      thNum--;
      while (thNum > 0) {
         if (th > tmps[thNum]) {
            th = tmps[thNum];
            break;
         }
         thNum--;
      }
      return th;
   }

   public void regularizeSlaveMeshACAP (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasMeanCurvatureNormal ()) { 
         System.out.println ("warning: no mean curvature info!");
         return;
      }

      PolygonalMesh mesh = null;
      if (info.getSlave () instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)info.getSlave ();
         if (mc.getMesh () instanceof PolygonalMesh) {
            mesh = (PolygonalMesh)mc.getMesh ();
         }
      }
      else if (info.getSlaveType () == SlaveType.RigidBody) {
         mesh = ((RigidBody)info.getSlave ()).getMesh ();
      }
      else {
         System.out.println ("warning: not polygonal-mesh-based info!");
         return;
      } 

      // ---initialize--- //
      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);
      VectorNd EdgeProp = new VectorNd (myFFD.getCtrlPntsNum()*3);
      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix Qua = new SparseBlockMatrix (rbs, rbs);
      SparseBlockMatrix EdgeQua = new SparseBlockMatrix (rbs, rbs);
      double c = 0;

      // ---get basis and laplacian matrix--- //
      SparseBlockMatrix Ls = info.getSymmetricLaplacianOperator ();
      if (Ls == null) {
         info.updateMeanCurvatureNormal ();
         Ls = info.getSymmetricLaplacianOperator ();
      }

      // ---evaluate local rotations for each cell--- //
      HashMap <Vertex3d, Matrix3d> RotMap = 
      new HashMap <Vertex3d, Matrix3d> ();

      HashMap <Vertex3d, Double> ScaleMap = 
      new HashMap <Vertex3d, Double> ();

      for (Vertex3d vtx : mesh.getVertices ()) {

         ScaledRigidTransform3d CellT = GeometryOperator.
         computeCellLocalConformalMap (vtx, info.getPointMap (), Ls);
         Matrix3d CellR = new Matrix3d (CellT.R);
         //TODO: to
         CellR.scale (CellT.s);
         RotMap.put (vtx, CellR);

         ScaleMap.put (vtx, new Double (CellT.s));
         /*
            LinkedHashSet<Vertex3d> neighbors = 
            GeometryOperator.getNeighborVetices (vtx);
            MatrixNd oldPos = new MatrixNd (neighbors.size (), 3);
            MatrixNd newPos = new MatrixNd (neighbors.size (), 3);
            int idx = 0;

            // find rotation R
            Iterator<Vertex3d> it = neighbors.iterator ();
            while  (it.hasNext ()) {
               Vertex3d v = it.next ();
               oldPos.setRow (idx, info.getPointUndeformedPosition (
                  v.getPosition ()));
               newPos.setRow (idx, info.getPointCurrentPosition (
                  v.getPosition ()));
               idx++;
            }
            AffineTransform3d affine = ARUNSVDOptimizer.
            fitRigid (oldPos, newPos, false);

            RotationMatrix3d R = new RotationMatrix3d ();
            R.set (affine.A);
            RotMap.put (vtx, R);*/
      }

      // ---loop through all edges--- // 
      HashSet<HalfEdge> VisitedEdges = new HashSet<HalfEdge> ();
      for (Face face : mesh.getFaces ()) {
         HalfEdge h0 = face.firstHalfEdge ();    
         HalfEdge he = h0;
         do {
            if (!VisitedEdges.contains (he)) {

               Vertex3d vtxI = he.getHead ();
               Vertex3d vtxJ = he.getTail ();
               SparseBlockMatrix Bi = slaveInfo.
               getBasis (vtxI.getPosition ());
               SparseBlockMatrix Bj = slaveInfo.
               getBasis (vtxJ.getPosition ());
               // Bi - Bj
               SparseBlockMatrix Bij = new SparseBlockMatrix (Bi);
               Bij.sub (Bj);

               // current position i, j
               Point3d curi = new Point3d ();
               curi.set (info.getPointCurrentPosition (
                  vtxI.getPosition()));
               Point3d curj = new Point3d ();
               curj.set (info.getPointCurrentPosition (
                  vtxJ.getPosition ()));

               // undeformed position i, j
               Point3d resti = new Point3d ();
               Point3d restj=  new Point3d ();
               resti.set (info.getPointUndeformedPosition (
                  vtxI.getPosition ()));
               restj.set (info.getPointUndeformedPosition (
                  vtxJ.getPosition ()));

               // edge ij
               Vector3d eij = new Vector3d ();
               Vector3d eji = new Vector3d ();
               eij.sub (curi, curj);
               eji.sub (curj, curi);
               Vector3d reij = new Vector3d ();
               Vector3d reji = new Vector3d ();
               reij.sub (resti, restj);
               reji.sub (restj, resti);

               // rotate undeform3d position i, j
               // test
               RotMap.get (vtxI).mul (reij);
               RotMap.get (vtxJ).mul (reji);

               // constant
               Vector3d consij = new Vector3d ();
               Vector3d consji = new Vector3d ();
               consij.sub (eij, reij);
               consji.sub (eji, reji);

               // weight w_ij
               double w_ij = Ls.getBlock (
                  vtxI.getIndex (), 
                  vtxJ.getIndex ()).get (0, 0);

               // TODO:
               double w_ji = w_ij * w_ij / info.getVoroniMassMatrix ().getBlock (
                  vtxJ.getIndex (), vtxJ.getIndex ()).get (0, 0) 
               / ScaleMap.get (vtxJ) / ScaleMap.get (vtxJ);
               w_ij = w_ij * w_ij / info.getVoroniMassMatrix ().getBlock (
                  vtxI.getIndex (), vtxI.getIndex ()).get (0, 0)
               / ScaleMap.get (vtxI) / ScaleMap.get (vtxI);


               // Quadratic term --- (Bi - Bj)^T(Bi - Bj)
               EdgeQua.mulTransposeLeftToBlkMat (Bij, Bij);
               EdgeQua.scale (w_ji + w_ij);

               // Proportional term Bij x consij
               Bij.mulTranspose (EdgeProp, new VectorNd (consij));
               EdgeProp.scale (w_ij);
               consji.negate ();
               VectorNd EdgePropji = new VectorNd ();
               Bij.mulTranspose(EdgePropji, new VectorNd (consji));
               EdgePropji.scale (w_ji);
               EdgeProp.add (EdgePropji);

               // constant term
               c += (consij.normSquared () * w_ij);
               c += (consji.normSquared () * w_ji);

               Qua.add (EdgeQua);
               Prop.add (EdgeProp);

               VisitedEdges.add (he);
               if (he.opposite != null) {
                  VisitedEdges.add (he.opposite);
               }
            }

            he = he.getNext ();
         } while (he != h0);

      }


      Qua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (Qua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {
         form.addQuadraticTerm (Qua);
         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }

   public void regularizeSlaveMeshARAP (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasMeanCurvatureNormal ()) { 
         System.out.println ("warning: no mean curvature info!");
         return;
      }

      PolygonalMesh mesh = null;
      if (info.getSlave () instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)info.getSlave ();
         if (mc.getMesh () instanceof PolygonalMesh) {
            mesh = (PolygonalMesh)mc.getMesh ();
         }
      }
      else if (info.getSlaveType () == SlaveType.RigidBody) {
         mesh = ((RigidBody)info.getSlave ()).getMesh ();
      }
      else {
         System.out.println ("warning: not polygonal-mesh-based info!");
         return;
      } 

      // ---initialize--- //
      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);
      VectorNd EdgeProp = new VectorNd (myFFD.getCtrlPntsNum()*3);
      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix Qua = new SparseBlockMatrix (rbs, rbs);
      SparseBlockMatrix EdgeQua = new SparseBlockMatrix (rbs, rbs);
      double c = 0;

      // ---get basis and laplacian matrix--- //
      SparseBlockMatrix Ls = info.getSymmetricLaplacianOperator ();
      if (Ls == null) {
         info.updateMeanCurvatureNormal ();
         Ls = info.getSymmetricLaplacianOperator ();
      }

      // ---evaluate local rotations for each cell--- //
      HashMap <Vertex3d, RotationMatrix3d> RotMap = 
      new HashMap <Vertex3d, RotationMatrix3d> ();

      for (Vertex3d vtx : mesh.getVertices ()) {

         RotationMatrix3d CellR = GeometryOperator.
         computeCellLocalRotation (vtx, info.getPointMap (), Ls);
         RotMap.put (vtx, CellR);
         /*
            LinkedHashSet<Vertex3d> neighbors = 
            GeometryOperator.getNeighborVetices (vtx);
            MatrixNd oldPos = new MatrixNd (neighbors.size (), 3);
            MatrixNd newPos = new MatrixNd (neighbors.size (), 3);
            int idx = 0;

            // find rotation R
            Iterator<Vertex3d> it = neighbors.iterator ();
            while  (it.hasNext ()) {
               Vertex3d v = it.next ();
               oldPos.setRow (idx, info.getPointUndeformedPosition (
                  v.getPosition ()));
               newPos.setRow (idx, info.getPointCurrentPosition (
                  v.getPosition ()));
               idx++;
            }
            AffineTransform3d affine = ARUNSVDOptimizer.
            fitRigid (oldPos, newPos, false);

            RotationMatrix3d R = new RotationMatrix3d ();
            R.set (affine.A);
            RotMap.put (vtx, R);*/
      }

      // ---loop through all edges--- // 
      HashSet<HalfEdge> VisitedEdges = new HashSet<HalfEdge> ();
      for (Face face : mesh.getFaces ()) {
         HalfEdge h0 = face.firstHalfEdge ();    
         HalfEdge he = h0;
         do {
            if (!VisitedEdges.contains (he)) {

               Vertex3d vtxI = he.getHead ();
               Vertex3d vtxJ = he.getTail ();
               SparseBlockMatrix Bi = slaveInfo.
               getBasis (vtxI.getPosition ());
               SparseBlockMatrix Bj = slaveInfo.
               getBasis (vtxJ.getPosition ());
               // Bi - Bj
               SparseBlockMatrix Bij = new SparseBlockMatrix (Bi);
               Bij.sub (Bj);

               // current position i, j
               Point3d curi = new Point3d ();
               curi.set (info.getPointCurrentPosition (
                  vtxI.getPosition()));
               Point3d curj = new Point3d ();
               curj.set (info.getPointCurrentPosition (
                  vtxJ.getPosition ()));

               // undeformed position i, j
               Point3d resti = new Point3d ();
               Point3d restj=  new Point3d ();
               resti.set (info.getPointUndeformedPosition (
                  vtxI.getPosition ()));
               restj.set (info.getPointUndeformedPosition (
                  vtxJ.getPosition ()));

               // edge ij
               Vector3d eij = new Vector3d ();
               Vector3d eji = new Vector3d ();
               eij.sub (curi, curj);
               eji.sub (curj, curi);
               Vector3d reij = new Vector3d ();
               Vector3d reji = new Vector3d ();
               reij.sub (resti, restj);
               reji.sub (restj, resti);

               // rotate undeform3d position i, j
               // test
               reij.transform (RotMap.get (vtxI));
               reji.transform (RotMap.get (vtxJ));

               // constant
               Vector3d consij = new Vector3d ();
               Vector3d consji = new Vector3d ();
               consij.sub (eij, reij);
               consji.sub (eji, reji);

               // weight w_ij
               double w_ij = Ls.getBlock (
                  vtxI.getIndex (), 
                  vtxJ.getIndex ()).get (0, 0);

               double w_ji = w_ij * w_ij / info.getVoroniMassMatrix ().getBlock (
                  vtxJ.getIndex (), vtxJ.getIndex ()).get (0, 0);
               w_ij = w_ij * w_ij / info.getVoroniMassMatrix ().getBlock (
                  vtxI.getIndex (), vtxI.getIndex ()).get (0, 0);


               // Quadratic term --- (Bi - Bj)^T(Bi - Bj)
               EdgeQua.mulTransposeLeftToBlkMat (Bij, Bij);
               EdgeQua.scale (w_ji + w_ij);

               // Proportional term Bij x consij
               Bij.mulTranspose (EdgeProp, new VectorNd (consij));
               EdgeProp.scale (w_ij);
               consji.negate ();
               VectorNd EdgePropji = new VectorNd ();
               Bij.mulTranspose(EdgePropji, new VectorNd (consji));
               EdgePropji.scale (w_ji);
               EdgeProp.add (EdgePropji);

               // constant term
               c += (consij.normSquared () * w_ij);
               c += (consji.normSquared () * w_ji);

               Qua.add (EdgeQua);
               Prop.add (EdgeProp);

               VisitedEdges.add (he);
               if (he.opposite != null) {
                  VisitedEdges.add (he.opposite);
               }
            }

            he = he.getNext ();
         } while (he != h0);

      }


      Qua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (Qua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {
         form.addQuadraticTerm (Qua);
         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }





   public void regularizeSlaveMeshBending (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasMeanCurvatureNormal ()) { 
         System.out.println ("warning: no mean curvature info!");
         return;
      }

      PolygonalMesh mesh = null;
      if (info.getSlave () instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)info.getSlave ();
         if (mc.getMesh () instanceof PolygonalMesh) {
            mesh = (PolygonalMesh)mc.getMesh ();
         }
      }
      else if (info.getSlaveType () == SlaveType.RigidBody) {
         mesh = ((RigidBody)info.getSlave ()).getMesh ();
      }
      else {
         System.out.println ("warning: not polygonal-mesh-based info!");
         return;
      } 

      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);
      VectorNd CellProp = new VectorNd (myFFD.getCtrlPntsNum()*3);
      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix Qua = new SparseBlockMatrix (rbs, rbs);
      SparseBlockMatrix CellQua = new SparseBlockMatrix (rbs, rbs);

      SparseBlockMatrix B = slaveInfo.getBasis ();
      SparseBlockMatrix Ls = info.getSymmetricLaplacianOperator ();
      if (Ls == null) {
         info.updateMeanCurvatureNormal ();
         Ls = info.getSymmetricLaplacianOperator ();
      }

      //info.updateConformalModes ();

      ScaledRigidTransform3d ctf = info.getConformalTransform ();

      int num = mesh.numVertices () - mesh.numDisconnectedVertices ();
      VectorNd oldPos = new VectorNd (num * 3);
      VectorNd newPos = new VectorNd (num * 3);
      VectorNd Dis = new VectorNd (num * 3);
      double c = 0;
      for (Vertex3d vtx : mesh.getVertices ()) {
         oldPos.setZero ();
         newPos.setZero ();
         Dis.setZero ();
         int idxI = vtx.getIndex ();
         Point3d pntI = new Point3d ();
         vtx.getWorldPoint (pntI);
         Point3d oldPntI = new Point3d ();
         oldPntI.set (
            info.getPointUndeformedPosition (
               vtx.getPosition ()));

         SparseBlockMatrix CellLs = info.
         getCellLaplacianOperator (vtx.getPosition ());
         if (CellLs == null) {
            info.updateCellLaplacian (vtx.getIndex ());
            CellLs = info.
            getCellLaplacianOperator (vtx.getPosition ());
         }


         Double CellArea = info.getCellVoronoiArea (
            vtx.getPosition ());
         double cellArea = CellArea;
         //CellArea *= (ctf.s * ctf.s);
         //cellArea *= ctf.s;

         // Ls^T M^-1 Ls
         SparseBlockMatrix invMLs = new SparseBlockMatrix (CellLs);
         invMLs.scale (1.0 / cellArea);
         SparseBlockMatrix LsinvMLs = new SparseBlockMatrix ();
         LsinvMLs.mulTransposeLeftToBlkMat (CellLs, invMLs);

         // warping mean curvature normal
         // R D(P)  = D(RP)
         RotationMatrix3d CellR = GeometryOperator.
         computeCellLocalRotation (vtx, info.getPointMap (), Ls);

         LinkedHashSet<Vertex3d> neighbors = 
         GeometryOperator.getNeighborVetices (vtx);
         Iterator <Vertex3d> it = neighbors.iterator ();
         while (it.hasNext ()) {
            Vertex3d vtxJ = it.next ();
            int idxJ = vtxJ.getIndex ();
            Point3d pntJ = new Point3d ();
            vtxJ.getWorldPoint (pntJ);
            Point3d oldPntJ = new Point3d ();
            oldPntJ.set (
               info.getPointUndeformedPosition (
                  vtxJ.getPosition ()));
            oldPntJ.transform (CellR);
            Vector3d dis = new Vector3d ();
            dis.sub (pntJ, oldPntJ);

            newPos.setSubVector (idxJ*3, pntJ);
            oldPos.setSubVector (idxJ*3, oldPntJ);
            Dis.setSubVector (idxJ*3, dis);
         }
         oldPntI.transform (CellR);
         newPos.setSubVector (idxI*3, pntI);
         oldPos.setSubVector (idxI*3, oldPntI);
         Vector3d dis = new Vector3d ();
         dis.sub (pntI, oldPntI);
         Dis.setSubVector (idxI*3, dis);

         // make quadratic term
         SparseBlockMatrix BTK = new SparseBlockMatrix ();
         BTK.mulTransposeLeftToBlkMat (B, LsinvMLs);
         CellQua.mulToBlkMat (BTK, B);

         // make proportional term
         CellProp.setZero ();
         BTK.mul (CellProp, Dis); 

         // make constant term  
         VectorNd tmp = new VectorNd ();
         // cur^T Ls^T M^-1 Ls cur
         LsinvMLs.mul (tmp, newPos);
         c += tmp.dot (newPos);
         // -2 cur^T Ls^T M^-1 Ls old
         LsinvMLs.mul (tmp, oldPos);
         c -= tmp.dot (newPos) * 2;
         // old^T Ls^T M^-1 Ls old
         c += tmp.dot (oldPos);

         Qua.add (CellQua);
         Prop.add (CellProp);
      }

      Qua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (Qua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {
         form.addQuadraticTerm (Qua);
         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }

   public void regularizeSlaveMeshLaplacianSmooth (
      SparseQuadLSFormulator form, SparseBlockMatrix H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasMeanCurvatureNormal ()) { 
         System.out.println ("warning: no mean curvature info!");
         return;
      }

      PolygonalMesh mesh = null;
      if (info.getSlave () instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)info.getSlave ();
         if (mc.getMesh () instanceof PolygonalMesh) {
            mesh = (PolygonalMesh)mc.getMesh ();
         }
      }
      else if (info.getSlaveType () == SlaveType.RigidBody) {
         mesh = ((RigidBody)info.getSlave ()).getMesh ();
      }
      else {
         System.out.println ("warning: not polygonal-mesh-based info!");
         return;
      }

      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);
      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix Qua = new SparseBlockMatrix (rbs, rbs);


      SparseBlockMatrix B = slaveInfo.getBasis ();
      SparseBlockMatrix K = info.getLTML ();
      if (K == null) {
         info.updateMeanCurvatureNormal ();
         K = info.getLTML ();
      }

      int num = mesh.numVertices () - mesh.numDisconnectedVertices ();
      VectorNd Pos = new VectorNd (num * 3);
      int idx = 0;
      for (Vertex3d vtx : mesh.getVertices ()) {
         Pos.setSubVector (idx * 3, vtx.getWorldPoint ());
         idx++;
      }

      // for constant term
      double c = 0;

      // make quadratic term
      SparseBlockMatrix BTK = new SparseBlockMatrix ();
      BTK.mulTransposeLeftToBlkMat (B, K);
      Qua.mulToBlkMat (BTK, B);

      // make proportional term
      BTK.mul (Prop, Pos);

      // make constant term
      VectorNd KPos = new VectorNd ();
      K.mul (KPos, Pos);
      c += KPos.dot (Pos);

      Qua.scale (weight);
      Prop.scale (weight);
      c *= weight;

      if (H != null) {
         H.set (Qua);
      }

      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {
         form.addQuadraticTerm (Qua);
         form.addPropotionalTerm (Prop); 
         form.addConstantTerm (c);
      }
   }

   /**
    * making virtual nodes stay at where they are
    * @param form
    * @param weight
    * 
    * @return Hessian matrix
    */
   public MatrixNd regularizeSubdomainGridByDisplace (
      DenseQuadLSFormulator form, double weight) {

      if (!myFFD.isSubdivisionEnabled ()) {
         System.out.println (
         "warning: FFD subdivision not enabled!");
      }
      int num = myFFD.numSubdomainCtrlPnts ();

      MatrixNd H = new MatrixNd (num*3, num*3);
      H.setIdentity ();

      if (weight != 1.0) {
         H.scale (weight);
      }

      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
   }

   /**
    * mapping goal: limiting ffd virtual edge length and direction change in each 
    * iteration; make quadratic term directly
    * 
    * @param form formulator, if not null this term will be added into it.
    * @param weight
    * 
    * @return Hessian matrix
    */
   public MatrixNd regularizeSubdomainGridByEdge (DenseQuadLSFormulator form, double weight) {

      if (!myFFD.isSubdivisionEnabled ()) {
         System.out.println (
         "warning: FFD subdivision not enabled!");
      }

      int num = myFFD.numSubdomainCtrlPnts ();

      MatrixNd H = new MatrixNd (num*3, num*3);
      int [][] edges = myFFD.getFFD ().edgeVtxIdx;

      for (int i = 0; i < edges.length; i++) { 
         if (!myFFD.getFFD ().isSubdomainCtrlPnt (edges[i][0])) {
            continue;
         }
         if (!myFFD.getFFD ().isSubdomainCtrlPnt (edges[i][1])) {
            continue;
         }
         int idx0 = myFFD.getFFD().fullToSubdomainCPMap(edges[i][0]);
         int idx1 = myFFD.getFFD().fullToSubdomainCPMap(edges[i][1]);

         for (int k = 0; k < 3; k++) {
            double val = H.get (idx0*3+k, idx0*3+k);
            H.set (idx0*3+k, idx0*3+k, val+1);

            val = H.get (idx1*3+k, idx1*3+k);
            H.set (idx1*3+k, idx1*3+k, val+1);

            val = H.get (idx1*3+k, idx0*3+k);
            H.set (idx1*3+k, idx0*3+k, val-1);

            val = H.get (idx0*3+k, idx1*3+k);
            H.set (idx0*3+k, idx1*3+k, val-1);
         }
      }


      if (weight != 1.0) {
         H.scale (weight);
      }

      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public void regularizeSubdomainWarpedSlaveStrain (
      DenseQuadLSFormulator form, MatrixNd H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return;
      }
      if (!info.hasStiffness ()) { 
         System.out.println ("warning: no stiffness info!");
         return;
      }
      if (!info.isElementWarpingEnabled ()) {
         System.out.println ("warning: no element warping info!");
         return;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return;
      }

      MatrixNd Quad = new MatrixNd (myFFD.numSubdomainCtrlPnts ()*3, 
         myFFD.numSubdomainCtrlPnts ()*3);
      VectorNd Prop = new VectorNd (myFFD.numSubdomainCtrlPnts ()*3);

      int [] rbs = new int [myFFD.numSubdomainCtrlPnts ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix qua = new SparseBlockMatrix (rbs, rbs);
      double c = 0;

      if (info.getSlaveType () == SlaveType.Fem) {
         // stiffness
         FemModel3d fem = (FemModel3d)info.getSlave ();

         for (FemElement3d ele : fem.getElements ()) {
            FemNode3d [] nodes = ele.getNodes (); 

            // detect if the element is inside the sub-domain 
            boolean inside = false;
            boolean [] marks = new boolean [ele.numNodes ()];
            int idx = -1;
            for (FemNode3d node : nodes) {
               boolean mark = slaveInfo.getFFDPoint (node.getPosition ()).
               isInsideSubdomain ();
               idx++;
               if (mark) {
                  inside = true;
                  marks[idx] = true;
                  continue;
               }
            }

            if (! inside) {
               // ignore the elements 
               // outside of the sub-domain
               continue;
            }

            VectorNd rest = new VectorNd (ele.numNodes ()*3);
            VectorNd current = new VectorNd (ele.numNodes () *3);
            RotationMatrix3d Rele = info.getElementRotation (ele);
            int i = 0;
            for (FemNode3d node : nodes) {
               Point3d re = new Point3d();
               re.set (info.
                  getPointUndeformedPosition (node.
                     getPosition ()));
               // warping start
               re.transform (Rele);
               // warping end
               rest.setSubVector (i*3, re);
               Point3d cu = 
               info.getPointCurrentPosition (node.getPosition ());
               current.setSubVector (i*3, cu);
               i++;
            }

            VectorNd dis = new VectorNd ();
            dis.sub (current, rest);



            // get K
            SparseBlockMatrix eleKrc = slaveInfo.createElementStiffnessMatrix (
               ele, (LinearMaterial)fem.getMaterial (), true);
            SparseBlockMatrix eleKc = removeRow (eleKrc, marks);
            // get B
            SparseBlockMatrix eleB = NFFDSlaveInfo.
            getElementSubdomainBasis (ele, slaveInfo);
            // get BTK
            SparseBlockMatrix BTKc = new SparseBlockMatrix ();
            BTKc.mulTransposeLeftToBlkMat (eleB, eleKc);

            // proportional term
            VectorNd pro = new VectorNd  ();
            BTKc.mul (pro, dis);

            // quadratic term
            SparseBlockMatrix quaEle = new SparseBlockMatrix ();
            SparseBlockMatrix BTK = removeCol (BTKc, marks);
            quaEle.mulToBlkMat (BTK, eleB);

            qua.add (quaEle);
            Prop.add (pro);
         }

         qua.scale (weight);
         Prop.scale (weight);
      }
      else {
         throw new UnsupportedOperationException (
         "Current version only support FEM slave!");
      }

      if (H != null) {
         H.set (qua);
      }
      else {
         Quad.set (qua);
      }
      if (q != null) {
         q.set (Prop);
      }

      if (form != null) {
         if (H != null) {
            form.addQuadraticTerm (H);
         }
         else {
            form.addQuadraticTerm (Quad);
         }
         form.addPropotionalTerm (Prop); 
      }
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public void regularizeSubdomainSlaveStrain (
      DenseQuadLSFormulator form, MatrixNd H, VectorNd q, 
      NFFDSlaveInfo slaveInfo, double weight) {

      SlaveInfo info = slaveInfo.getSlaveInfo ();
      if (! myFFD.isKnownSlave (info)) {
         throw new IllegalArgumentException ("Not my slave!");
      }
      if (!info.hasPoint ()) { 
         System.out.println ("warning: no point info!");
         return;
      }
      if (!info.hasStiffness ()) { 
         System.out.println ("warning: no stiffness info!");
         return;
      }
      if (!slaveInfo.isBTKSavingEnabled ()) {
         System.out.println ("warning: no BTK info!");
         return;
      }
      if (!slaveInfo.isElementBTKSavingEnabled ()) {
         System.out.println ("warning: no element BTK info!");
         return;
      }

      MatrixNd Quad = new MatrixNd ();

      int [] rbs = new int [myFFD.numSubdomainCtrlPnts ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix SQua = new SparseBlockMatrix (rbs, rbs);
      VectorNd prop = new VectorNd  ();
      double c = 0;

      if (info.getSlaveType () == SlaveType.Fem) {
         // stiffness
         FemModel3d fem = (FemModel3d)info.getSlave ();

         // basis
         // rest position
         // TODO: need modify for bad nodes
         VectorNd rest = new VectorNd (fem.numNodes ()*3);
         VectorNd current = new VectorNd (fem.numNodes () *3);
         boolean [] marks = new boolean [fem.numNodes ()];
         int i = 0;
         //stopwatch.checkpoint ("initilization");
         boolean inside = false;
         for (FemNode3d node : fem.getNodes ()) {
            //System.out.println (node.getSolveIndex ());
            Point3d re = new Point3d (
               info.getPointUndeformedPosition (node.getPosition ()));
            rest.setSubVector (i*3, re);
            Point3d cu = 
            info.getPointCurrentPosition (node.getPosition ());
            current.setSubVector (i*3, cu);


            if (slaveInfo.getFFDPoint (node.getPosition ()).
            isInsideSubdomain ()) {
               inside = true;
               marks[i] = true;
            }
            i++;
         }
         VectorNd dis = new VectorNd ();
         dis.sub (current, rest);

         if (!inside) {
            return;
         }


         // this is based assumption that
         // stiffness matrix created in  
         // nodes order;
         SparseBlockMatrix Krc = info.getStiffnessMatrix ();
         SparseBlockMatrix Kc = removeRow (Krc, marks); 
         SparseBlockMatrix B = slaveInfo.getSubdomainBasis ();
         // make matrix
         SparseBlockMatrix BTKc = new SparseBlockMatrix ();
         BTKc.mulTransposeLeftToBlkMat (B, Kc);

         BTKc.mul (prop, dis);

         SparseBlockMatrix BTK = removeCol (BTKc, marks);
         //SparseBlockMatrix K = removeCol (Kc, marks);
         //BTKc.mulTransposeLeftToBlkMat (B, K);
         SQua.mulToBlkMat (BTK, B);

         SQua.scale (weight);
         prop.scale (weight);
         c *= weight;
      }
      else {
         throw new UnsupportedOperationException (
         "Current version only support FEM slave!");
      }

      if (H != null) {
         H.set (SQua);
      }
      else {
         Quad.set (SQua);
      }
      if (q != null) {
         q.set (prop);
      }

      if (form != null) {
         if (H != null) {
            form.addQuadraticTerm (H);
         }
         else {
            form.addQuadraticTerm (Quad);
         }
         form.addPropotionalTerm (prop); 
      }
   }


   private static SparseBlockMatrix removeRow (SparseBlockMatrix K, boolean [] marks) {
      if (K.numBlockRows () != marks.length) {
         throw new ImproperSizeException ("Interanl Error");
      }

      int num = 0;
      for (boolean mark : marks) {
         if (mark) {
            num++;
         }
      }

      int [] br = new int [num];
      int [] bc = new int [K.numBlockCols ()];
      for (int i = 0; i < num; i++) {
         br[i] = 3;
      }
      for (int j = 0; j < bc.length; j++) {
         bc[j] = 3;
      }
      SparseBlockMatrix rK = new SparseBlockMatrix (br, bc);

      int idx = 0;
      for (int i = 0; i < K.numBlockRows (); i++) {
         if (!marks[i]) {
            continue;
         }

         MatrixBlock blk = K.firstBlockInRow (i);
         while (blk != null) {
            rK.addBlock (idx, blk.getBlockCol (), blk.clone ());
            blk = blk.next ();
         }
         idx++;
      }

      return rK;
   }

   private static SparseBlockMatrix removeCol (SparseBlockMatrix K, boolean [] marks) {
      if (K.numBlockCols () != marks.length) {
         throw new ImproperSizeException ("Interanl Error");
      }

      int num = 0;
      for (boolean mark : marks) {
         if (mark) {
            num++;
         }
      }

      int [] br = new int [K.numBlockRows ()];
      int [] bc = new int [num];
      for (int i = 0; i < br.length; i++) {
         br[i] = 3;
      }
      for (int j = 0; j < bc.length; j++) {
         bc[j] = 3;
      }
      SparseBlockMatrix rK = new SparseBlockMatrix (br, bc);

      int idx = 0;
      for (int i = 0; i < K.numBlockCols (); i++) {
         if (!marks[i]) {
            continue;
         }

         MatrixBlock blk = K.firstBlockInCol (i);
         while (blk != null) {
            rK.addBlock (blk.getBlockRow (), idx, blk.clone ());
            blk = blk.down ();
         }
         idx++;
      }

      return rK;
   }
}
