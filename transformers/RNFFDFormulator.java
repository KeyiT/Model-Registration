package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.Arrays;
import java.util.Iterator;
import java.util.Set;
import java.util.Map.Entry;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.LinearMaterial;
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
import artisynth.models.swallowingRegistrationTool.utilities.FEMQualityUtilities;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.ScaledRigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;

public class RNFFDFormulator implements Formulator{

   protected ReducedNFFDeformer myFFD;

   public RNFFDFormulator () {
   }

   public RNFFDFormulator (ReducedNFFDeformer ffd) {
      myFFD = ffd;
   }

   @Override
   public void setTransformer (RegistrationTransformer tf) {
      if (tf instanceof ReducedNFFDeformer) {
         myFFD = (ReducedNFFDeformer)tf;
         return;
      }
      throw new ClassCastException ("Incompatible transformer!");
   }

   public ReducedNFFDeformer getTransformer () {
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
      int num = myFFD.getCtrlPntsNum ();

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
   public MatrixNd regularizeGridByEdge (DenseQuadLSFormulator form, double weight) {
      int num = myFFD.getCtrlPntsNum ();

      MatrixNd H = new MatrixNd (num*3, num*3);
      int [][] edges = myFFD.getFFD ().edgeVtxIdx;

      for (int i = 0; i < edges.length; i++) { 
         if ( (!myFFD.getFFD ().isActivePointIndex (edges[i][0])) ||
         (!myFFD.getFFD ().isActivePointIndex (edges[i][1]))) {
            continue;
         }
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

      if (weight != 1.0) {
         H.scale (weight);
      }

      if (form != null) {
         form.addQuadraticTerm (H);
      }

      return H;
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
      if (!(info.hasPoint () && info.isPointSavingEnabled ())) { 
         System.out.println ("warning: no stiffness info!");
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
   public void regularizeSlaveStrain (
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
   }

   /**
    * 
    * @param form
    * @param H
    * @param q
    * @param slaveInfo
    * @param weight
    */
   public void regularizeWarpedSlaveStrain (
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

      MatrixNd Quad = new MatrixNd (myFFD.getCtrlPntsNum()*3, myFFD.getCtrlPntsNum()*3);
      VectorNd Prop = new VectorNd (myFFD.getCtrlPntsNum()*3);

      int [] rbs = new int [myFFD.getCtrlPntsNum()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix qua = new SparseBlockMatrix (rbs, rbs);
      double c = 0;

      if (info.getSlaveType () == SlaveType.Fem) {
         // stiffness
         FemModel3d fem = (FemModel3d)info.getSlave ();

         SparseBlockMatrix B = slaveInfo.getBasis ();

         for (FemElement3d ele : fem.getElements ()) {
            FemNode3d [] nodes = ele.getNodes ();
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

            SparseBlockMatrix BTK = slaveInfo.getElementBTK (ele);
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
               (LinearMaterial)fem.getMaterial (), true);
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
         w = w/Math.pow (Se, 2);

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
            // warping end

            rest.setSubVector (ii*3, re);
            Point3d cu = 
            info.getPointCurrentPosition (node.getPosition ());
            current.setSubVector (ii*3, cu);
            ii++;
         }
         dis.sub (current, rest);
         bia.sub (current, ideal);

         SparseBlockMatrix BTK = slaveInfo.getElementBTK (ele);
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


}
