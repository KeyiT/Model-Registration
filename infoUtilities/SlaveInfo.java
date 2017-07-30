package artisynth.models.swallowingRegistrationTool.infoUtilities;

import java.io.InvalidClassException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;


import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.FemUtilities;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.FixedMeshBody;
import artisynth.core.mechmodels.MeshComponent;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.TransformableGeometry;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.infoUtilities.SlaveInfo.SlaveType;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import artisynth.models.swallowingRegistrationTool.transformers.RegistrationTransformer;
import artisynth.models.swallowingRegistrationTool.utilities.ElementWarpingUtilities;
import artisynth.models.swallowingRegistrationTool.utilities.GeometryOperator;
import maspack.geometry.MeshBase;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RotationMatrix3d;
import maspack.matrix.ScaledRigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.util.Clonable;

public class SlaveInfo extends ObjInfo{

   TransformableGeometry mySlave;
   SlaveType myType = SlaveType.Unknown;

   private boolean hasEdge = false;
   private boolean hasPoint = false;
   private boolean hasStiffness = false;
   private boolean hasElement = false;
   private boolean hasMeanCurvatureNormal = false;

   private boolean saveEdge = false;
   private boolean savePoint = false;
   private boolean updateUndeformedPoint = false;
   private boolean updateCfMd = false;
   private boolean saveStiffness = false;
   private boolean saveElement = false;
   private boolean saveElementStiffness = false;
   private boolean updateMeanCurvatureNormal = false;
   private boolean updateCellLs = false;

   CloudInfo<EdgeInfo> edgesInfo;
   // slave points --> undeformed position
   Map <Point3d, Point3d> myPointMap = 
   new LinkedHashMap <Point3d, Point3d> ();

   private boolean posConsis = true;
   protected SparseBlockMatrix Stiffness;
   
   // mean curvature normal vector
   // which is the result of Laplace-Beltrami operator 
   // on triangular 2-manifold mesh
   protected VectorNd Kh;
   protected SparseBlockMatrix Ls;
   protected SparseBlockMatrix LsSym;
   protected SparseBlockMatrix Mass;
   protected SparseBlockMatrix LsTinvMLs;
   
   LinkedHashMap <Point3d, SparseBlockMatrix> CellLaplacian =
   new LinkedHashMap <Point3d, SparseBlockMatrix> ();
   LinkedHashMap <Point3d, Double> CellVoronoiArea = new 
   LinkedHashMap <Point3d, Double> ();

   LinkedHashMap <Object, ScaledRigidTransform3d> ElementRotations = 
   new LinkedHashMap <Object, ScaledRigidTransform3d> ();
   LinkedHashMap <Point3d, RotationMatrix3d> PointRotations = 
   new LinkedHashMap <Point3d, RotationMatrix3d> ();
   
   LinkedHashMap <Object, SparseBlockMatrix> ElementKs = 
   new LinkedHashMap <Object, SparseBlockMatrix> ();
   
   // transform current position to undeformed position
   // in least-square sense
   ScaledRigidTransform3d ConformalTransform;
   
   boolean warpElements = false;
   boolean warpPoints = false;

   private HashMap<RegistrationTransformer, TransformerSlaveInfo> mySubInfos = new 
   HashMap <RegistrationTransformer, TransformerSlaveInfo> ();

   public static enum SlaveType {
      /**
       * slave type is finite-element 
       * model
       */
      Fem,
      /**
       * slave type is rigid body
       */
      RigidBody,
      /**
       * slave type is mesh component, 
       * including skin mesh or fixed 
       * mesh.
       */
      MeshBody,
      /**
       * slave type is muscle, including
       * point-to-point muscle, multi-point
       * muscle, muscle bundle;
       */
      Muscle,
      /**
       * slave is a single point
       */
      Point,
      /**
       * slave is a instance of <code>MeshBase</code>
       */
      Mesh,
      /**
       * slave type is not recognized
       */
      Unknown
   }

   public void setSlave (TransformableGeometry slave) {
      mySlave = slave;
      hasEdge = checkForEdge(slave);
      hasPoint = checkForPoint(slave);
      hasStiffness = checkForStiffness (slave);
      hasElement = checkForElement (slave);
      hasMeanCurvatureNormal = checkForMeanCurvatureNormal (slave);
      if (slave instanceof RigidBody) {
         myType = SlaveType.RigidBody;
         posConsis = false;
      }
      else if (slave instanceof MeshComponent) {
         myType = SlaveType.MeshBody;
         posConsis = false;
      }
      else if (slave instanceof FemModel3d) {
         FemModel3d fem = (FemModel3d) slave;
         fem.resetRestPosition ();
         myType = SlaveType.Fem;
      }
      else if (slave instanceof Point){
         myType = SlaveType.Point;
      }
   }

   public void setSlave (MeshBase mesh) {
      FixedMeshBody slave = new FixedMeshBody ();
      slave.setPose (mesh.getMeshToWorld ());
      slave.setMesh (mesh);
      slave.getMesh ().setFixed (false);
      mySlave = slave;
      hasEdge = checkForEdge(slave);
      hasPoint = checkForPoint(slave);
      myType = SlaveType.Mesh;
      posConsis = false;
   }

   public TransformableGeometry getSlave () {
      return mySlave;
   }

   public <T extends TransformableGeometry> T getSlave (Class<T> slaveType) {
      return slaveType.cast (mySlave);
   }

   public SlaveType getSlaveType () {
      if (mySlave == null) {
         throw new ImproperStateException ("Slave not initialized!");
      }
      return myType;
   }

   public TransformerSlaveInfo addTransformerInfo (
      RegistrationTransformer tf) {

      TransformerSlaveInfo info =  tf.
      createSubSlaveInfo (mySlave, this);

      if (info ==  null) {
         throw new NullPointerException (
         "This transformer is not slave assignable!");
      }
      return mySubInfos.put (tf, info);
   }

   public TransformerSlaveInfo getTransformerInfo (
      RegistrationTransformer tf) {

      return mySubInfos.get (tf);
   }

   public void getTransformerInfos (Set<TransformerSlaveInfo> infos) {
      infos.addAll (mySubInfos.values ());
   }

   /**
    * if the slave is class or subclass of {@link MeshComponent} and has
    * polygonal mesh, or {@link RigidBody} or {@link FemModel3d}, 
    * return true, otherwise return false
    * @param slave for checking
    * @return 
    */
   public boolean checkForEdge (TransformableGeometry slave) {
      if (MeshComponent.class.isAssignableFrom (slave.getClass ())) {
         MeshComponent body = (MeshComponent) slave;
         if (PolygonalMesh.class.isAssignableFrom (body.getMesh ().getClass ())) {
            return true;
         }
      }
      else if (FemModel3d.class.isAssignableFrom (slave.getClass ())) {
         return true;
      }
      else if (RigidBody.class.isAssignableFrom (slave.getClass ())) {
         RigidBody rb = (RigidBody)slave;
         if (rb.getMesh () != null) {
            return true;
         }
      }
      return false;
   }

   /**
    * if the slave of this info has edge return true, 
    * otherwise return false
    * @return 
    */
   public boolean hasEdge () {
      return hasEdge;
   }

   public void enableEdgeSaving (boolean enable) {
      saveEdge = enable;
   }

   public boolean isEdgeSavingEnabled () {
      return saveEdge;
   }

   /**
    * @return a cloud-info which has an EdgeInfo list
    */
   public CloudInfo<EdgeInfo> getEdgeCloudInfo() {
      return edgesInfo;
   }

   public List<EdgeInfo> getEdgeInfos () {
      List<EdgeInfo> infos = new ArrayList<EdgeInfo>();
      edgesInfo.getInfos (infos);
      return infos;
   }


   /**
    * if the slave has edge, 
    * make edge-infos for this salve;
    * if the slave dose not have edge throw an
    * new exception.
    */
   public void makeEdgeInfo () {
      if (! hasEdge()) {
         try {
            throw new InvalidClassException ("Slave dose not have edges");
         }
         catch (InvalidClassException e) {
            e.printStackTrace();
         }
      }
      edgesInfo = new CloudInfo<EdgeInfo> ();

      if (FemModel3d.class.isAssignableFrom (mySlave.getClass ())) {
         EdgeAllocator.makeEdgeInfo(edgesInfo, (FemModel3d)mySlave);
      } 
      else if (RigidBody.class.isAssignableFrom (mySlave.getClass ())){
         RigidBody rb = (RigidBody)mySlave;
         EdgeAllocator.makeEdgeInfo(edgesInfo, rb.getMesh ());
      }
      else if (MeshComponent.class.isAssignableFrom (mySlave.getClass ())) {
         MeshComponent body = (MeshComponent)mySlave;
         if (PolygonalMesh.class.isAssignableFrom (body.getMesh ().getClass ())) {
            EdgeAllocator.makeEdgeInfo(edgesInfo, (PolygonalMesh)body.getMesh ());
         }
      }
      System.out.println ("make " + edgesInfo.myInfoList.size () + " edges !!");
   }

   /**
    * if the slave is a class or subclass of {@link MeshComponent}, 
    * {@link RigidBody}, {@link FemModel3d} or {@link Point}, 
    * return true, otherwise return false
    * @return 
    */
   public boolean hasPoint () {
      return hasPoint;
   }

   /**
    * if the <tt>slave</tt> is class or subclass of {@link MeshComponent}, 
    * {@link RigidBody}, {@link FemModel3d} or {@link Point}, 
    * return true, otherwise return false
    * @param slave for checking
    * @return 
    */
   public boolean checkForPoint (TransformableGeometry slave) {
      if (MeshComponent.class.isAssignableFrom (slave.getClass ())) {
         return true;
      }
      else if (FemModel3d.class.isAssignableFrom (slave.getClass ())) {
         return true;
      }
      else if (RigidBody.class.isAssignableFrom (slave.getClass ())) {
         return true;
      }
      else if (slave instanceof Point) {
         return true;
      }
      return false;
   }

   public void enablePointSaving (boolean enable) {
      savePoint = enable;
   }

   public boolean isPointSavingEnabled () {
      return savePoint;
   }

   public void enableUndeformedPointUpdating (boolean enable) {
      updateUndeformedPoint = enable;
   }

   public boolean isUndeformedPointUpdatingEnabled () {
      return updateUndeformedPoint;
   }

   /**
    * if {@link #checkForPoint} is true, points of slave will be saved
    * as a key set of hash map. The default value of this hash map is 
    * undeformed position;
    */
   public void savePoints () {
      if (! hasPoint()) {
         try {
            throw new InvalidClassException ("Slave dose not have points");
         }
         catch (InvalidClassException e) {
            e.printStackTrace();
         }
      }
      if (FemModel3d.class.isAssignableFrom (mySlave.getClass ())) {
         savePoints(myPointMap, (FemModel3d)mySlave);
      } 
      else if (RigidBody.class.isAssignableFrom (mySlave.getClass ())){
         RigidBody rb = (RigidBody)mySlave;
         savePoints(myPointMap, rb.getMesh ());
      }
      else if (MeshComponent.class.isAssignableFrom (mySlave.getClass ())) {
         MeshComponent body = (MeshComponent)mySlave;
         savePoints(myPointMap, body.getMesh ());
      }
      else if (mySlave instanceof Point) {
         savePoints(myPointMap, (Point)mySlave);
      }

      System.out.println ("saved " + myPointMap.size () + " points !!");
   }

   private void savePoints (Map<Point3d, Point3d> map, MeshBase mesh) {
      map.clear ();
      for (Vertex3d vtx: mesh.getVertices ()) {
         Point3d pnt = vtx.getPosition ();
         map.put (pnt, null);
      }
   }

   private void savePoints (Map<Point3d, Point3d> map, FemModel3d fem) {
      map.clear ();
      for (FemNode3d node : fem.getNodes ()) {
         // ignore bad nodes
         if (node.numAdjacentElements () == 0) {
            continue;
         }
         Point3d pnt = node.getPosition ();
         map.put (pnt, null);
      }
      
   }

   private void savePoints (Map<Point3d, Point3d> map, Point pnt) {
      map.clear ();
      map.put (pnt.getPosition (), null);
   }


   /**
    * Advance slave's undeformed position to current position.
    */
   public void savePointCurretAsUndeformed () {
      if (myPointMap.size () == 0) {
         throw new ImproperStateException ("point hash map is empty");
      }
      Set<Point3d> keys = myPointMap.keySet ();
      for (Point3d key : keys) {
         Point3d val = getPointCurrentPosition (key);
         myPointMap.put (key, val);
      }
      
      if (saveElementStiffness) {
         updateElemenKs ();
      }
      
      if (isElementWarpingEnabled ()) {
         resetElementRotations ();
      }

      System.out.println ("State updated!");
   }

   public Point3d getPointCurrentPosition (Point3d pnt) {
      if (!myPointMap.containsKey (pnt)) {
         throw new NullPointerException ("Point does not belong to this slave!");
      }
      Point3d rp = new Point3d ();
      rp.set (pnt);
      if (!posConsis) {
         if (RigidBody.class.isAssignableFrom (mySlave.getClass ())){
            RigidBody rb = (RigidBody)mySlave;
            if (!rb.getMesh ().meshToWorldIsIdentity ()) {
               rp.transform (rb.getMesh ().XMeshToWorld);
            }
         }
         else if (MeshComponent.class.isAssignableFrom (mySlave.getClass ())) {
            MeshComponent body = (MeshComponent)mySlave;
            if (!body.getMesh ().meshToWorldIsIdentity ()) {
               rp.transform (body.getMesh ().XMeshToWorld);
            }
         }
      }
      return rp;
   }

   public void setPointWorldPosition (Point3d pnt, Point3d newPosition) {
      if (!myPointMap.containsKey (pnt)) {
         throw new NullPointerException ("Point does not belong to this slave!");
      }
      Point3d rp = new Point3d ();
      rp.set (newPosition);
      if (!posConsis) {
         if (RigidBody.class.isAssignableFrom (mySlave.getClass ())){
            RigidBody rb = (RigidBody)mySlave;
            if (!rb.getMesh ().meshToWorldIsIdentity ()) {
               rp.inverseTransform (rb.getMesh ().XMeshToWorld);
            }
         }
         else if (MeshComponent.class.isAssignableFrom (mySlave.getClass ())) {
            MeshComponent body = (MeshComponent)mySlave;
            if (!body.getMesh ().meshToWorldIsIdentity ()) {
               rp.inverseTransform (body.getMesh ().XMeshToWorld);
            }
         }
      }
      pnt.set (rp);
   }

   public Point3d getPointUndeformedPosition (Point3d pnt) {
      if (myPointMap.size () == 0) {
         throw new ImproperStateException ("point hash map is empty");
      }
      if (!myPointMap.containsKey (pnt)) {
         throw new NullPointerException ("Point does not belong to this slave!");
      }
      return myPointMap.get (pnt);
   }

   public Collection<Point3d> getUndeformedPositions () {
      return myPointMap.values ();
   }

   public Set<Point3d> getPoints () {
      return myPointMap.keySet ();
   }

   public Map<Point3d, Point3d> getPointMap () {
      return myPointMap;
   }

   public boolean checkForStiffness (TransformableGeometry slave) {
      if (slave instanceof FemModel3d) {
         return true;
      }
      return false;
   }

   public boolean hasStiffness () {
      return hasStiffness;
   }

   public void enableStiffnessSaving (boolean enable) {
      saveStiffness = enable;
   }

   public boolean isStiffnessSavingEnabled () {
      return saveStiffness;
   }

   /**
    * only apply to FEM slave
    */
   public void updateStiffness () {
      if (myType != SlaveType.Fem) {
         throw new UnsupportedOperationException (
         "Slave does not have stiffness");
      }

      FemModel3d fem = (FemModel3d)mySlave;
      fem.setMaterial (new LinearMaterial(200.0, 0.2, false));
      for (int i = 0; i < fem.numNodes (); i++) {
         FemNode3d node = fem.getNode (i);
         node.setRestPosition (getPointUndeformedPosition (
            node.getPosition ()));
         //System.out.println (getPointUndeformedPosition (
         //node.getPosition ()));
      }

      //System.out.println ("updating Stiffness start! " );
      fem.updateStressAndStiffness ();

      Stiffness = new SparseBlockMatrix (
         fem.getActiveStiffness ());
      //System.out.println ("updating Stiffness end! " );
      Stiffness.scale (-1.0);
      if (warpPoints) {
         warpStiffnessMatrix ();
      }
   }

   /**
    * R x K x R^T
    * @return 
    * if point warping map size was not equal to stiffness matrix, 
    * return stiffness matrix without warping;
    */
   public void warpStiffnessMatrix () {
      if (Stiffness.numBlockRows () > PointRotations.size ()) {
         return;
      }

      SparseBlockMatrix R = createPointRotationMatrix ();

      SparseBlockMatrix RK = new SparseBlockMatrix ();
      RK.mulToBlkMat (R, Stiffness);
      Stiffness.mulTransposeRightToBlkMat (RK, R);
   }

   /**
    * only apply to FEM slave
    */
   public SparseBlockMatrix createStiffnessMatrix (boolean warp) {
      if (myType != SlaveType.Fem) {
         throw new UnsupportedOperationException (
         "Slave does not have stiffness");
      }

      FemModel3d fem = (FemModel3d)mySlave;
      fem.setMaterial (new LinearMaterial(200.0, 0.2, false));
      for (int i = 0; i < fem.numNodes (); i++) {
         FemNode3d node = fem.getNode (i);
         node.setRestPosition (getPointUndeformedPosition (
            node.getPosition ()));
      }

      fem.updateStressAndStiffness ();

      SparseBlockMatrix K = new SparseBlockMatrix (
         fem.getActiveStiffness ());

      K.scale (-1.0);
      if (warp) {
         if (K.numBlockRows () > PointRotations.size ()) {
            return K;
         }

         SparseBlockMatrix R = createPointRotationMatrix ();

         SparseBlockMatrix RK = new SparseBlockMatrix ();
         RK.mulToBlkMat (R, K);
         K.mulTransposeRightToBlkMat (RK, R);
      }

      return K;
   }

   public SparseBlockMatrix getStiffnessMatrix () {
      return Stiffness;
   }



   public boolean checkForElement (TransformableGeometry slave) {
      if (slave instanceof FemModel3d) {
         return true;
      }
      return false;
   }

   public boolean hasElement () {
      return hasElement;
   }

   public void enableElementSaving (boolean enable) {
      saveElement = enable;
   }

   public boolean isElementSavingEnabled () {
      return saveElement;
   }

   // slave elements --> rest elements
   HashMap <Object, Point3d []> myElements = new HashMap<Object, Point3d []>();

   /**
    * only apply to FEM slave
    */
   public void updateElements () {
      if (myType != SlaveType.Fem) {
         throw new UnsupportedOperationException (
         "Slave does not have elements");
      }

      FemModel3d fem = (FemModel3d)mySlave;
      myElements.clear ();
      for (FemElement3d ele : fem.getElements ()) {
         Point3d [] pnts = new Point3d [ele.numNodes ()];
         for (int i = 0; i < ele.numNodes (); i++) {
            Point3d pnt = new Point3d (ele.getPoint (i));
            pnts[i] = pnt;
         }
         myElements.put (ele, pnts);
      }
   }

   public Point3d[] updateElement (FemElement3d ele) {
      Point3d [] rests = new Point3d [ele.numNodes ()];
      for (int i = 0; i < rests.length; i++) {
         rests[i] = new Point3d (ele.getPoint (i));
      }
      return myElements.put (ele, rests);
   }

   public Point3d[] getRestElement (Object ele) {
      return myElements.get (ele);
   }

   public void getRestElement (Point3d [] restR, Object ele) {
      Point3d [] rests = myElements.get (ele);
      if (restR.length < rests.length) {
         throw new IllegalArgumentException ("Incompatible size");
      }
      int idx = 0;
      for (Point3d rest : rests) {
         restR[idx++] = new Point3d (rest);
      }
   }

   public void enableElementWarping (boolean enable) {
      warpElements = enable;
   }

   public boolean isElementWarpingEnabled () {
      return warpElements;
   }

   public void enablePointWarping (boolean enable) {
      warpPoints = enable;
   }

   public boolean isPointWarpingEnabled () {
      return warpPoints;
   }

   public RotationMatrix3d getElementRotation (Object ele) {
      if (!ElementRotations.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find the element!");
      }
      return ElementRotations.get (ele).R;
   }
   
   public void setElementRotation (Object ele, RotationMatrix3d R) {
      if (!ElementRotations.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find the element!");
      }
      
      ElementRotations.get (ele).R.set (R);
   }

   public double getElementScaling (Object ele) {
      if (!ElementRotations.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find the element!");
      }
      return ElementRotations.get (ele).s;
   }
   
   public void setElementScaling (Object ele, double s) {
      if (!ElementRotations.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find the element!");
      }
      
      ElementRotations.get (ele).setScale (s);
   }
   
   public SparseBlockMatrix getElementStiffness (Object ele) {
      if (!ElementRotations.containsKey (ele)) {
         throw new IllegalArgumentException (
         "Failed to find the element!");
      }
      return ElementKs.get (ele);
   }

   public RotationMatrix3d getPointRotation (Point3d pnt) {
      if (!PointRotations.containsKey (pnt)) {
         throw new IllegalArgumentException (
         "Failed to find the point!");
      }
      return PointRotations.get (pnt);
   }

   public SparseBlockMatrix createPointRotationMatrix () {
      int [] rbs = new int [PointRotations.size ()];
      for (int i = 0; i < rbs.length; i++) {
         rbs[i] = 3;
      }
      SparseBlockMatrix R = new SparseBlockMatrix(rbs, rbs);
      R.setVerticallyLinked (true);
      Set<Entry<Point3d, Point3d>> entry = myPointMap.entrySet ();
      Iterator it = entry.iterator ();
      int idx = 0;
      while (it.hasNext ()) {
         Entry<Point3d, Point3d> me = 
         (Entry<Point3d, Point3d>)it.next ();
         Matrix3x3Block blk = new Matrix3x3Block ();
         blk.set (PointRotations.get (me.getKey ()));
         R.addBlock (idx, idx, blk);
         idx++;
      }
      return R;
   }

   // TODO: need modify
   public void updateElementRotations () {
      if (!hasElement ()) {
         throw new UnsupportedOperationException (
         "slave does not have elements");
      }

      if (getSlaveType () == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlave;
         for (int i = 0; i < fem.numNodes (); i++) {
            FemNode3d node = fem.getNode (i);
            node.setRestPosition (getPointUndeformedPosition (
               node.getPosition ()));
         }

         for (int ii = 0; ii < fem.numElements (); ii++) {
            FemElement3d ele = fem.getElement (ii);
            ScaledRigidTransform3d Re = new ScaledRigidTransform3d ();
            ElementWarpingUtilities.evalWarpingRotation (Re, ele);
            ElementRotations.put (ele, Re);
         }
      }
   }
   
   /**
    * make every local rotation as identity matrix
    */
   public void resetElementRotations () {
      if (!hasElement ()) {
         throw new UnsupportedOperationException (
         "slave does not have elements");
      }

      if (getSlaveType () == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlave;
         for (int i = 0; i < fem.numNodes (); i++) {
            FemNode3d node = fem.getNode (i);
            node.setRestPosition (getPointUndeformedPosition (
               node.getPosition ()));
         }

         for (int ii = 0; ii < fem.numElements (); ii++) {
            FemElement3d ele = fem.getElement (ii);
            ScaledRigidTransform3d Re = new ScaledRigidTransform3d ();
            ElementRotations.put (ele, Re);
         }
      }
   }
   
   public void enableElementStiffnessSaving (boolean enable) {
      this.saveElementStiffness = enable;
   }
   
   public boolean isElementStiffnessSavingEnabled () {
      return this.saveElementStiffness;
   }
   
   public void updateElemenKs () {
      if (!hasElement ()) {
         throw new UnsupportedOperationException (
         "slave does not have elements");
      }

      if (getSlaveType () == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlave;
         for (int i = 0; i < fem.numNodes (); i++) {
            FemNode3d node = fem.getNode (i);
            node.setRestPosition (getPointUndeformedPosition (
               node.getPosition ()));
         }

         for (int ii = 0; ii < fem.numElements (); ii++) {
            FemElement3d ele = fem.getElement (ii);
            ElementKs.put (ele, createElementStiffnessMatrix (
               ele, (LinearMaterial)fem.getMaterial (), false));
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
      eleK.setVerticallyLinked (true);

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
         assembleWarpingMatrix (getElementRotation (ele), ele);
         
         VectorNd rest = new VectorNd (ele.numNodes ()*3);
         VectorNd current = new VectorNd (ele.numNodes ()*3);
         MatrixNd Current = new MatrixNd (ele.numNodes (), 3);
         MatrixNd Rest = new MatrixNd (ele.numNodes (), 3);
         int ii = 0;
         for (FemNode3d node : nodes) {
            Point3d re = 
            getPointUndeformedPosition (node.getPosition ());
            Point3d cu = 
            getPointCurrentPosition (node.getPosition ());
            rest.setSubVector (ii*3, re);
            current.setSubVector (ii*3, cu);
            Current.setRow (ii, cu);
            ii++;
         }

         SparseBlockMatrix RtK = new SparseBlockMatrix ();
         RtK.mulToBlkMat (RMat, eleK);
         
         VectorNd rkx = new VectorNd (ele.numNodes ()*3);
         RtK.mul (rkx, rest);
         ii = 0;
         for (FemNode3d node : nodes) {
            Point3d re = new Point3d ();
            rkx.getSubVector (ii*3, re);
            Rest.setRow (ii, re);
            ii++;
         }
         
         RotationMatrix3d inR = new RotationMatrix3d ();
         inR.set (ARUNSVDOptimizer.fitRigid (Rest, Current, false).A);
         inR.mul (getElementRotation (ele));
         setElementRotation (ele, inR);
         
         RMat = ElementWarpingUtilities.
         assembleWarpingMatrix (inR, ele);
         
         VectorNd kcur = new VectorNd ();
         eleK.mul (kcur, current);
         double s = kcur.dot (current);
         s = s / kcur.dot (rest);
         setElementScaling (ele, s);
         
         RtK.mulToBlkMat (RMat, eleK);
         eleK.mulTransposeRightToBlkMat (RtK, RMat);
      }
      return eleK;
   }

   public void updatePointRotations () {
      if (!hasPoint ()) {
         throw new UnsupportedOperationException (
         "slave does not have points");
      }
      if (!hasElement ()) {
         throw new UnsupportedOperationException (
         "slave does not have elements");
      }

      PointRotations.clear ();

      if (getSlaveType () == SlaveType.Fem) {
         FemModel3d fem = (FemModel3d)mySlave;
         for (int i = 0; i < fem.numNodes (); i++) {
            FemNode3d node = fem.getNode (i);
            node.setRestPosition (getPointUndeformedPosition (
               node.getPosition ()));
         }

         for (int ii = 0; ii < fem.numNodes (); ii++) {
            FemNode3d node = fem.getNode (ii);
            if (node.numAdjacentElements () == 0) {
               continue;
            }
            RotationMatrix3d Re = new RotationMatrix3d ();
            ElementWarpingUtilities.evalWarpingRotation (Re, node);
            PointRotations.put (node.getPosition (), Re);
         }
      }
   }
   
   public static boolean checkForMeanCurvatureNormal (TransformableGeometry slave) {
      if (slave instanceof RigidBody) {
         RigidBody rb = (RigidBody) slave;
         PolygonalMesh mesh = rb.getMesh ();
         if (mesh.isManifold () && mesh.isTriangular () 
         && mesh.numDegenerateFaces () == 0) {
            return true;
         }
      }
      else if (slave instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)slave;
         if (mc.getMesh () instanceof PolygonalMesh) {
            PolygonalMesh mesh = (PolygonalMesh) mc.getMesh ();
            if (mesh.isManifold () && mesh.isTriangular ()
            && mesh.numDegenerateFaces () == 0) {
               return true;
            }
         }
      }
      return false;
   }

   public boolean hasMeanCurvatureNormal () {
      return hasMeanCurvatureNormal;
   }
   
   public void enableMeanCurvatureNormalSaving () {
      updateMeanCurvatureNormal = true;
   }
   
   public boolean isMeanCurvatureNormalUpdateEnabled () {
      return updateMeanCurvatureNormal;
   }
   
   public void updateMeanCurvatureNormal () {
      if (! hasMeanCurvatureNormal) {
         throw new UnsupportedOperationException (
            "No mean curvature normal info for slave!");
       }
      
      PolygonalMesh mesh = null;
      if (myType == SlaveType.RigidBody) {
         RigidBody rb = (RigidBody) mySlave;
         mesh = rb.getMesh ();
      }
      if (mySlave instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)mySlave;
         mesh = (PolygonalMesh) mc.getMesh ();
      }
      
      Mass = new SparseBlockMatrix ();
      Ls = new SparseBlockMatrix ();
      
      Kh = GeometryOperator.
      computeMeanCurvatureNormal (Ls, Mass, mesh, true);
      LsSym = GeometryOperator.createLaplacianMapMatrix (mesh, false);
      
      SparseBlockMatrix invM = new SparseBlockMatrix (Mass);
      GeometryOperator.invertMassMatrix (invM);
      
      SparseBlockMatrix invMLs = new SparseBlockMatrix ();
      invMLs.mulToBlkMat (invM, Ls);
      LsTinvMLs = new SparseBlockMatrix ();
      LsTinvMLs.mulTransposeLeftToBlkMat (Ls, invMLs);
   }
   
   public void updateCellLaplacian (int idx) {
      if (! hasMeanCurvatureNormal) {
         throw new UnsupportedOperationException (
            "No mean curvature normal info for slave!");
       }
      
      PolygonalMesh mesh = null;
      if (myType == SlaveType.RigidBody) {
         RigidBody rb = (RigidBody) mySlave;
         mesh = rb.getMesh ();
      }
      if (mySlave instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)mySlave;
         mesh = (PolygonalMesh) mc.getMesh ();
      }
      
      Vertex3d vtx =  mesh.getVertex (idx);
      if (vtx == null) {
         throw new IllegalArgumentException (
            "index out of bound");
      }
      
      SparseBlockMatrix cellLs = GeometryOperator.
      createLaplacianMapMatrix (mesh, vtx, true);
      
      Double CellArea = GeometryOperator.computeVoronoiArea (vtx);
      
      CellLaplacian.put (vtx.getPosition (), cellLs);
      CellVoronoiArea.put (vtx.getPosition (), CellArea);
   }
   
   public void updateCellLaplacian () {
      if (! hasMeanCurvatureNormal) {
         throw new UnsupportedOperationException (
            "No mean curvature normal info for slave!");
       }
      
      PolygonalMesh mesh = null;
      if (myType == SlaveType.RigidBody) {
         RigidBody rb = (RigidBody) mySlave;
         mesh = rb.getMesh ();
      }
      if (mySlave instanceof MeshComponent) {
         MeshComponent mc = (MeshComponent)mySlave;
         mesh = (PolygonalMesh) mc.getMesh ();
      }
      
      for (int i = 0; i < 
      (mesh.numVertices () - mesh.numDisconnectedVertices ()); 
      i++) {
         Vertex3d vtx =  mesh.getVertex (i);
         
         SparseBlockMatrix cellLs = GeometryOperator.
         createLaplacianMapMatrix (mesh, vtx, true);
         
         Double CellArea = GeometryOperator.computeVoronoiArea (vtx);
         
         CellLaplacian.put (vtx.getPosition (), cellLs);
         CellVoronoiArea.put (vtx.getPosition (), CellArea);
      }
   }
   
   public VectorNd getMeanCurvatureNormal () {
      return Kh;
   }
   
   public void getMeanCurvatureNormal (VectorNd K) {
      if (Kh == null) {
         throw new NullPointerException (
            "Mean Curvatrue normal info not saved");
      }
      
      K.set (Kh);
   }
   
    public SparseBlockMatrix getLaplacianOperator () {
       return Ls;
    }
    
    public SparseBlockMatrix getSymmetricLaplacianOperator () {
       return LsSym;
    }
    
    public SparseBlockMatrix getVoroniMassMatrix () {
       return Mass;
    }
    
    public SparseBlockMatrix getLTML () {
       return LsTinvMLs;
    }
    
    public SparseBlockMatrix getCellLaplacianOperator (Point3d pnt) {
       return CellLaplacian.get (pnt);
    }
    
    public double getCellVoronoiArea (Point3d pnt) {
       return CellVoronoiArea.get (pnt);
    }

   public void enableConformalModesUpdate (boolean enable) {
      updateCfMd = enable;
   }

   public boolean isConformalModesUpdateEnabled () {
      return updateCfMd;
   }

   public void updateConformalModes () {
      MatrixNd srcData = new MatrixNd ();
      MatrixNd tgtData = new MatrixNd ();

      Set ens = myPointMap.entrySet ();
      Iterator it = ens.iterator ();

      srcData.setSize (myPointMap.size (), 3);
      tgtData.setSize (myPointMap.size (), 3);
      int idx = 0;
      while (it.hasNext ()) {
         Entry<Point3d, Point3d> me = 
         (Entry<Point3d, Point3d>)it.next ();

         srcData.setRow (idx, me.getValue ());
         tgtData.setRow (idx, getPointCurrentPosition (
            me.getKey ()));
         idx++;
      }

      AffineTransform3d affine =
      new AffineTransform3d ();
      
      double s = ARUNSVDOptimizer.fitRigid (affine, srcData, tgtData, false);
      ConformalTransform = new ScaledRigidTransform3d ();
      ConformalTransform.p.set (affine.p);
      ConformalTransform.R.set (affine.A);
      affine.A.scale (s);
      ConformalTransform.s = s;
      
      it = ens.iterator ();
      idx = 0;
      while (it.hasNext ()) {
         Entry<Point3d, Point3d> me = 
         (Entry<Point3d, Point3d>)it.next ();
         me.getValue ().transform (affine);
         idx++;
      }
      
      
      for (Double area: CellVoronoiArea.values ()) {
         area *= (s * s);
      }
      if (Mass != null) {
         Mass.scale (s * s);
      }
      if (LsTinvMLs != null) {
         LsTinvMLs.scale (1.0 / s / s);
      }
   }
   
   public ScaledRigidTransform3d getConformalTransform () {
      return ConformalTransform;
   }


   public boolean update () {
      boolean flag = selfUpdate ();

      for (TransformerSlaveInfo info : mySubInfos.values ()) {
         flag &= info.update ();
      }

      return (flag & super.update ());
   }

   public boolean selfUpdate () {
      if (mySlave != null) {
         hasEdge = checkForEdge (mySlave);
         hasPoint = checkForPoint (mySlave);
         hasStiffness = checkForStiffness (mySlave);
         hasElement = checkForElement (mySlave);
      }

      if (savePoint && hasPoint) {
         savePoints();
      }
      if (saveEdge && hasEdge) {
         makeEdgeInfo ();
      }
      if (savePoint && hasPoint & updateCfMd) {
         updateConformalModes ();
      }
      if (updateUndeformedPoint && hasPoint) {
         savePointCurretAsUndeformed ();
      }
      if (hasElement && saveElement) {
         updateElements ();
      }
      if (hasElement && warpElements) {
         // TODO
         //resetElementRotations ();
      }
      if (hasPoint && hasElement && warpPoints) {
         updatePointRotations ();
      }
      if (hasStiffness && saveStiffness) {
         updateStiffness();
      }
      if (hasMeanCurvatureNormal && updateMeanCurvatureNormal) {
         updateMeanCurvatureNormal ();
      }
      if (hasMeanCurvatureNormal && updateCellLs) {
         updateCellLaplacian ();
      }
      return true;
   }

   public boolean update (RegistrationTransformer tf) {
      TransformerSlaveInfo slaveInfo = getTransformerInfo (tf);
      if (slaveInfo == null) {
         throw new NullPointerException ("Not my transformer!");
      }

      return slaveInfo.update ();
   }

}
