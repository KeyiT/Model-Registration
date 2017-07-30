package artisynth.models.swallowingRegistrationTool.transformers;

import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.ComponentChangeEvent;
import artisynth.core.modelbase.CompositeComponent;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.util.ScanToken;
import artisynth.core.workspace.RootModel;
import artisynth.models.modelOrderReduction.Matrix3x3DiagBlock;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.utilities.*;
import maspack.geometry.AABBTree;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Boundable;
import maspack.geometry.MeshBase;
import maspack.geometry.PointMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;
import maspack.properties.HierarchyNode;
import maspack.properties.Property;
import maspack.properties.PropertyInfoList;
import maspack.render.RenderList;
import maspack.util.NumberFormat;
import maspack.util.ReaderTokenizer;
/**
 * 
 * @author KeyiTang
 *
 */
public class NFFD3d extends NURBSVolume implements ModelComponent{

   protected Point3d myBBEndVertex = new Point3d ();
   protected Matrix3d myUVT;
   protected ControlGridMethod myGridMethod = ControlGridMethod.PCA;
   protected SplineType myType = SplineType.NonRational;

   protected double [][][] myWeights;

   private int [] knotsLength = new int [3];

   // embedded meshes
   protected ArrayList <MeshBase> myMeshes = new ArrayList<MeshBase> ();
   private HashMap<MeshBase, HashMap<Point3d, NFFDPoint3d>> myMeshFFDPnts = 
   new HashMap<MeshBase, HashMap<Point3d, NFFDPoint3d>> ();
   private HashMap<Point3d, NFFDPoint3d> myFFDPnts = new HashMap<Point3d, NFFDPoint3d> ();

   PointList<Point> myCtrlPntAgents = new PointList<Point> (
   Point.class, "NFFD3d Control Points", "NCPs");

   AABBTree myAABBTree;
   boolean myBVTreeValid = false;
   ArrayList<NFFD3dControlUnit> myUnits = new ArrayList<NFFD3dControlUnit> ();

   public enum SplineType {
      Rational,
      NonRational,
      Bezier
   }

   public enum ControlGridMethod {
      /**
       * create parallelpiped control grid 
       * based on PCA of embedded mesh; This
       * is default method;
       */
      PCA,
      /**
       * this method would create 
       * parallelepiped based on 
       * pose and and end vertex
       */
      Null
   }

   public NFFD3d () {
      super ();
      setDrawControlShape (false);
   }

   public NFFD3d (int [] numCtrlPnts, int [] degrees) {
      this ();
      setDegrees (degrees);
      setCtrlPntsNum (numCtrlPnts);
   }

   /**
    * initialize NFFD by set control grid pose and end vertex;
    * this method will set grid building method as 
    * {@link ControlGridMethod#Null}
    * @param numCtrlPnts
    * @param degrees
    * @param ctrlGridPose
    * @param BBEndVertex
    */
   public NFFD3d (int [] numCtrlPnts, int [] degrees, 
      RigidTransform3d ctrlGridPose, Point3d BBEndVertex) {
      this ();
      setDegrees (degrees);
      setCtrlPntsNum (numCtrlPnts);
      myXObjToWorld.set (ctrlGridPose);
      myBBEndVertex.set (BBEndVertex);
      myGridMethod = ControlGridMethod.Null;
      createControlGrid ();
   }

   public NFFD3d (int [] numCtrlPnts, int [] degrees, Collection<MeshBase> meshes) {
      this ();
      setDegrees (degrees);
      setCtrlPntsNum (numCtrlPnts);
      embedMeshes (meshes);
      createControlGrid ();
      advanceMeshInitialStates ();
   }

   public NFFD3d (int [] numCtrlPnts, int [] degrees, MeshBase meshes) {
      this ();
      setDegrees (degrees);
      setCtrlPntsNum (numCtrlPnts);
      embedMesh (meshes);
      createControlGrid ();
      advanceMeshInitialStates ();
   }

   public void setCtrlPntPosition (int idx, Point3d pos) {
      if (idx >= myCtrlPntAgents.size () || idx < 0) {
         throw new IllegalArgumentException ("Index is outof range");
      }
      Point cp = myCtrlPntAgents.get (idx);
      cp.setPosition (pos);
      myBVTreeValid = false;
      mySubBVTreeValid = false;
   }

   public void setCtrlPntPosition (int uIdx, int vIdx, int tIdx, Point3d pos) {
      if (uIdx >= numCtrlPntsUVT[0] || vIdx >= numCtrlPntsUVT[1] || 
      tIdx >= numCtrlPntsUVT[2]) {
         throw new IllegalArgumentException ("Index is out of range");
      }

      setCtrlPntPosition (indexMap(uIdx, vIdx, tIdx), pos);
   }

   public void setCtrlPntPositions (MatrixNd P) {
      if (P.colSize () != 3) {
         throw new ImproperSizeException("Incompatible size");
      }
      if (P.rowSize () != myCtrlPnts.size ()) {
         throw new ImproperSizeException ("Incompatible size");
      }
      Point3d pos= new Point3d();

      for (int i = 0; i < P.rowSize (); i++) {
         pos.x = P.get (i, 0);
         pos.y = P.get (i, 1);
         pos.z = P.get (i, 2);
         setCtrlPntPosition (i, pos);
      }
   }

   public void setCtrlPntPositions (VectorNd P) {
      if (P.size() != (3*myCtrlPnts.size ())) {
         throw new ImproperSizeException("Incompatible size");
      }
      Point3d pos= new Point3d();

      for (int i = 0; i < myCtrlPnts.size (); i++) {
         pos.x = P.get (i*3);
         pos.y = P.get (i*3+1);
         pos.z = P.get (i*3+2);
         setCtrlPntPosition (i, pos);
      }
   }

   //----------------------------------------- Building NFFD Control Grid -----------------------------------------------//

   public void setCtrlGridMethod (ControlGridMethod method) {
      myGridMethod = method;
   }

   public ControlGridMethod getCtrlGridMethod () {
      return myGridMethod;
   }

   //TODO
   public void createControlGrid () {
      double [][] knots = new double [3][];

      // create knots vector
      if (myType == SplineType.Bezier) {
         for (int i = 0; i < 3; i++) {
            if (degreesUVT[i] != numCtrlPntsUVT[i]) {
               throw new ImproperStateException (
                  "Basis degrees must be the same as the number of "
                  + "control points for Bezier volume!");
            }
         }
      }
      for (int i = 0; i < 3; i++) {
         knots[i] = createUniformKnotsVector (degreesUVT[i], numCtrlPntsUVT[i]);
         knotsLength[i] = knots[i].length;
      }

      // build grid
      if (myGridMethod != ControlGridMethod.Null) {
         if (!computeBBPose ()) {
            throw new ImproperStateException (
               "Failed to create control grid in method " + 
               myGridMethod.toString ()  + "!");
         }
      }
      myCtrlPnts.clear ();
      if (myType != SplineType.Rational) {
         myUVT = createControlPoints (myCtrlPnts, myBBEndVertex, myXObjToWorld, numCtrlPntsUVT, null);
      }
      else {
         myUVT = createControlPoints (myCtrlPnts, myBBEndVertex, myXObjToWorld, numCtrlPntsUVT, myWeights);
      }
      set (degreesUVT, knots, getControlPoints ());
      buildControlUnitList ();
      myBVTreeValid = false;
      mySubBVTreeValid = false;
      updateControlPointAgents ();
      makeEdgePointIndex();
   }

   /**
    * create control grid based on vertices of <tt>referenceMesh</tt>
    * @param referenceMesh
    */
   public void createControlGrid (MeshBase referenceMesh) {
      double [][] knots = new double [3][];

      // create knots vector
      if (myType == SplineType.Bezier) {
         for (int i = 0; i < 3; i++) {
            if (degreesUVT[i] != numCtrlPntsUVT[i]) {
               throw new ImproperStateException (
                  "Basis degrees must be the same as the number of "
                  + "control points for Bezier volume!");
            }
         }
      }
      for (int i = 0; i < 3; i++) {
         knots[i] = createUniformKnotsVector (degreesUVT[i], numCtrlPntsUVT[i]);
         knotsLength[i] = knots[i].length;
      }

      // build grid
      if (myGridMethod != ControlGridMethod.Null) {
         computeBBPose (myXObjToWorld, myBBEndVertex, referenceMesh);
      }
      myCtrlPnts.clear ();
      if (myType != SplineType.Rational) {
         myUVT = createControlPoints (myCtrlPnts, myBBEndVertex, myXObjToWorld, numCtrlPntsUVT, null);
      }
      else {
         myUVT = createControlPoints (myCtrlPnts, myBBEndVertex, myXObjToWorld, numCtrlPntsUVT, myWeights);
      }

      set (degreesUVT, knots, getControlPoints ());
      buildControlUnitList ();
      myBVTreeValid = false;
      mySubBVTreeValid = false;
      updateControlPointAgents ();
      makeEdgePointIndex();
   }

   /**
    * re-create the control point agents according to 
    * current control point positions
    */
   protected void updateControlPointAgents () {
      myCtrlPntAgents.clear ();
      for (Vector4d cp4d : myCtrlPnts) {
         Point cp = new Point (new Point3d (cp4d.x, cp4d.y, cp4d.z));
         myCtrlPntAgents.add (cp);
      }
   }

   /**
    * pass control point agent position information to 
    * the corresponding control point.
    */
   public void updateGridState () {
      if (myCtrlPnts.size () != myCtrlPntAgents.size ()) {
         throw new ImproperStateException ("Incompatible control point size!");
      }
      int idx = 0;
      for (Vector4d cp : myCtrlPnts) {
         Point3d cpa = myCtrlPntAgents.get (idx++).getPosition ();
         for (int i = 0; i < 3; i++) {
            cp.set (i, cpa.get (i));
         }
      }
      myBVTreeValid = false;
      mySubBVTreeValid = false;
   }

   public PointList<Point> getCtrlPntAgents () {
      return myCtrlPntAgents;
   }

   public void getCtrlPntAgents (List<Point> list) {
      list.addAll (myCtrlPntAgents);
   }

   public void enableGridControl (RootModel root) {
      root.addRenderable (myCtrlPntAgents);
   }



   protected static Matrix3d createControlPoints (ArrayList <Vector4d> ctrlPnts, Point3d endVertex, 
      RigidTransform3d Pose, int [] numCtrlPnts, double [][][] weights) {

      if (numCtrlPnts.length != 3) {
         throw new IllegalArgumentException (
         "Dimension must be 3!");
      }

      // check weights
      if (weights != null) {
         if (weights.length != numCtrlPnts[0]) {
            throw new IllegalArgumentException (
            "Incompatible weight size!");
         }
         for (int i=0; i < numCtrlPnts[0] ; i ++) {
            if (weights[i].length != numCtrlPnts[1]) {
               throw new IllegalArgumentException (
               "Incompatible weight size!");
            }
            for (int j = 0; j < numCtrlPnts[1]; j++) {
               if (weights[i][j].length != numCtrlPnts[2]) {
                  throw new IllegalArgumentException (
                  "Incompatible weight size!");
               }
            }
         }
      }


      // generate local box
      Point3d [] pnts = new Point3d [2];
      pnts [0] = new Point3d ();
      pnts [1] = new Point3d (endVertex);
      pnts [1].inverseTransform (Pose);

      Point3d [] BBVs = new Point3d [8];
      int g = 0;
      for (int i = 0; i <=1 ; i++) {
         for (int j = 0; j <= 1; j++) {
            for (int k = 0; k <= 1; k++) {
               BBVs[g] = new Point3d ();
               BBVs[g].set (pnts[i].x, pnts[j].y, pnts[k].z);
               BBVs[g].transform (Pose);
               g++;
            }
         }
      }
      Matrix3d UVT = new Matrix3d ();
      Vector3d U = new Vector3d ();
      Vector3d V = new Vector3d ();
      Vector3d T = new Vector3d ();
      U.sub (getBBV(1, 0, 0, BBVs),  BBVs[0]);
      UVT.setColumn (0, U);
      V.sub (getBBV(0, 1, 0, BBVs),  BBVs[0]);
      UVT.setColumn (1, V);
      T.sub (getBBV(0, 0, 1, BBVs),  BBVs[0]);
      UVT.setColumn (2, T);
      
      ArrayList <Vector4d> cps = new ArrayList<Vector4d> ();
      for (int i=0; i < numCtrlPnts[0] ; i ++) {
         for (int j = 0; j < numCtrlPnts[1]; j++) {
            for (int k = 0; k < numCtrlPnts[2]; k++) {
               //System.out.println("setControlPoints: temInd is " + temInd);
               Vector3d u = new Vector3d (U);
               Vector3d v = new Vector3d (V);
               Vector3d t = new Vector3d (T);
               u.scale ((double)i/(double)(numCtrlPnts[0]-1));
               v.scale ((double)j/(double)(numCtrlPnts[1]-1));
               t.scale ((double)k/(double)(numCtrlPnts[2]-1));
               Vector4d cp = new Vector4d ();
               for (int ii = 0; ii < 3; ii++) {
                  cp.set (ii, u.get (ii)+v.get (ii)+t.get (ii) + BBVs[0].get (ii));
               }
               if (weights == null) {
                  cp.set (3, 1.0);
               }
               else {
                  cp.set (3, weights[i][j][k]);
               }
               cps.add (cp);
            }
         }
      }
      ctrlPnts.addAll (cps);

      return UVT;
   }

   public Matrix3d getUVTMat () {
      return myUVT;
   }

   /**
    * reset control grid based on grid pose and U, V, T 
    * vector3d;
    */
   public void resetControlGrid () {
      Vector3d U = new Vector3d ();
      Vector3d V = new Vector3d ();
      Vector3d T = new Vector3d ();
      myUVT.getColumn (0, U);
      myUVT.getColumn (1, V);
      myUVT.getColumn (2, T);
      int idx = 0;
      for (int i=0; i < numCtrlPntsUVT[0] ; i ++) {
         for (int j = 0; j < numCtrlPntsUVT[1]; j++) {
            for (int k = 0; k < numCtrlPntsUVT[2]; k++) {
               Vector3d u = new Vector3d (U);
               Vector3d v = new Vector3d (V);
               Vector3d t = new Vector3d (T);
               u.scale ((double)i/((double)numCtrlPntsUVT[0]-1));
               v.scale ((double)j/((double)numCtrlPntsUVT[1]-1));
               t.scale ((double)k/((double)numCtrlPntsUVT[2]-1));
               Point cp = myCtrlPntAgents.get (idx++);
               
               Point3d pos = new Point3d ();
               for (int ii = 0; ii < 3; ii++) {
                  pos.set (ii, u.get (ii)+v.get (ii)+t.get (ii) + myXObjToWorld.p.get (ii));
               }
               cp.setPosition (pos);
            }
         }
      }
   }

   private static Point3d getBBV(int argIndX, int argIndY, int argIndZ, Point3d [] pnts) {
      String tem;
      tem = Integer.toString (argIndX);
      tem = tem + Integer.toString (argIndY);
      tem = tem + Integer.toString (argIndZ);
      //System.out.println ("getBBV: tem is : " + Integer.valueOf (tem, 2));
      Point3d bv = new Point3d ();
      bv.set (pnts[Integer.valueOf (tem, 2)]);
      return bv;
   }

   public void setBBPose (RigidTransform3d XObjToWorld) {
      myXObjToWorld.set (XObjToWorld);
   }


   public Point3d getBBEndVertex () {
      return myBBEndVertex;
   }

   public void getBBEndVertex (Point3d bbEnd) {
      bbEnd.set (myBBEndVertex);
   }

   public void setBBEndVertex (Point3d end) {
      myBBEndVertex.set (end);
   }

   public boolean computeBBPose () {
      if (myMeshes.size () != 0) {
         PointMesh pm = new PointMesh();
         for (MeshBase mesh : myMeshes) {
            for (Vertex3d vtx : mesh.getVertices ()) {
               pm.addVertex (new Point3d (vtx.getWorldPoint ()));
            }
         }
         computeBBPose (myXObjToWorld, myBBEndVertex, pm);
         return true;
      }
      return false;
   }


   // TODO: may add more method later
   public void computeBBPose (RigidTransform3d pose, Point3d endVertex, MeshBase argMesh) {
      if (myGridMethod == ControlGridMethod.PCA)
         pose.set (PCA.computeBBVs (argMesh, endVertex, null));
   }

   //----------------------------------------- Embedding ------------------------------------//

   /**
    * from physical space to local box coordinates
    * @param argX point in physical space
    * @return u v t
    */
   public Point3d findPoint(Point3d pntInWorld) {

      if (myUVT == null) {
         throw new ImproperStateException ("NFFD Grid not initialized!");
      }

      /*
      Point3d uvt=  new Point3d (pntInWorld);
      uvt.inverseTransform (myXObjToWorld);
      for (int i = 0; i < 3; i++) {
         int num = knotsLength[i];
         Vector3d U = new Vector3d ();
         myUVT.getColumn (i, U);
         uvt.set (i, uvt.get (i) * KnotsUVT[i][KnotsUVT[i].length-1] / U.norm ());
      }*/

      NFFD3dControlUnit unit = findContainingUnit(pntInWorld);

      if (unit == null) {
         System.out.println ("Control unit not found! [" + pntInWorld + "]");
         return null;
      }

      int [][] ranges = unit.getKnotSpanRange ();
      Point3d uvt = new Point3d ();
      for (int i = 0; i < 3; i++) {
         double lower = KnotsUVT[i][ranges[i][0]];
         double upper = KnotsUVT[i][ranges[i][1]];
         uvt.set (i, (lower + upper)/2.0);
      }

      if (findUVT (uvt, pntInWorld, uvt) < 0) {
         System.out.println ("uvt not found! [" + pntInWorld + "]");
         return null;
      }
      
      // TODO: test
      //if (uvt.norm () < 1E-8) {
         //System.out.println(findUVT (uvt, pntInWorld, uvt));
      //}
      return uvt;
   }




   public void buildControlUnitList () {
      myUnits.clear ();
      for (int i = 0; i < numCtrlPntsUVT[0]-1; i++) {
         for (int j = 0; j < numCtrlPntsUVT[1]-1; j++) {
            for (int k = 0; k < numCtrlPntsUVT[2]-1; k++) {
               NFFD3dControlUnit unit = new NFFD3dControlUnit (this);
               int [] ids = {i, j, k};
               unit.setCtrlVertices (ids);
               myUnits.add (unit);
            }
         }
      }
   }

   protected BVTree getBVTree() {
      if (myAABBTree == null || !myBVTreeValid) {
         myAABBTree = new AABBTree();
         myAABBTree.setMaxLeafElements (2);
         Boundable[] elements = new Boundable[myUnits.size ()];
         for (int i = 0; i < elements.length; i++) {
            elements[i] = myUnits.get(i);
         }
         myAABBTree.build(elements, myUnits.size ());
         myBVTreeValid = true;
      }
      return myAABBTree;
   }


   /**
    * Returns the control unit within the NFFD that contains a specified
    * point, or <code>null</code> if there is no such element.
    * 
    * @param pnt Point for which containing unit is desired.
    * @return containing unit, or null.
    */
   public NFFD3dControlUnit findContainingUnit(Point3d pnt) {
      BVTree bvtree = getBVTree();
      ArrayList<BVNode> nodes = new ArrayList<BVNode>();
      //System.out.println (pnt);
      bvtree.intersectPoint(nodes, pnt);
      //System.out.println ("num nodes " + nodes.size());
      if (nodes.size() == 0) {
         return null;
      }
      for (BVNode n : nodes) {
         Boundable[] elements = n.getElements();
         for (int i = 0; i < elements.length; i++) {
            ((NFFD3dControlUnit)elements[i]).generateConvexHull ();
            if (((NFFD3dControlUnit)elements[i]).isInsideConvexHull (pnt)) {
               return (NFFD3dControlUnit)elements[i];
            }
         }
      }
      return null;
   }

   //----------------------------------------- Evaluation ------------------------------------//

   public Matrix3d evalJac (Vector3d r) {
      Point3d uvt = findPoint (new Point3d(r));
      if (uvt == null) {
         System.err.println ("Faild to find UVT coordinates!");
         return null;
      }
      Matrix3d Jac = evalJacUVT (uvt);
      Matrix3d restJac = evalRestJacUVT (uvt);
      Jac.mulInverse (restJac);
      return Jac;
   }


   /**
    * @return
    */
   public Matrix3d evalRestJacobian (Point3d r) {
      Point3d uvt = findPoint (r);
      if (uvt == null) {
         System.err.println ("Faild to find UVT coordinates!");
         return null;
      }

      Point3d [][][] pnts = new Point3d [numCtrlPntsUVT[0]][numCtrlPntsUVT[1]][numCtrlPntsUVT[2]];
      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[1]; j++) {
            for (int k = 0; k < numCtrlPntsUVT[2]; k++) {
               pnts[i][j][k] = new Point3d (restCtrlPnts[indexMap(i, j, k)]);
            }
         }
      }
      Matrix3d Jac = evalJacUVT (uvt, pnts);
      return Jac;
   }

   public Matrix3d evalRestJacUVT (Point3d uvt) {

      Point3d [][][] pnts = new Point3d 
      [numCtrlPntsUVT[0]][numCtrlPntsUVT[1]][numCtrlPntsUVT[2]];
      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[1]; j++) {
            for (int k = 0; k < numCtrlPntsUVT[2]; k++) {
               pnts[i][j][k] = new Point3d (restCtrlPnts[indexMap(i, j, k)]);
            }
         }
      }
      Matrix3d Jac = evalJacUVT (uvt, pnts);
      return Jac;
   }

   public boolean isPolyhedralConvex () {
      Point3d [][][] pnts = new Point3d 
      [numCtrlPntsUVT[0]][numCtrlPntsUVT[1]][numCtrlPntsUVT[2]];
      
      int idx = 0;
      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[1]; j++) {
            for (int k = 0; k < numCtrlPntsUVT[2]; k++) {
               Vector4d cp = getControlPoint (idx++);
               pnts[i][j][k] = new Point3d (cp.x, cp.y, cp.z);
               //System.out.println (pnts[i][j][k]);
            }
         }
      }
      
      for (Vector4d pnt4d : myCtrlPnts) {
         Point3d pnt = new Point3d (pnt4d.x, pnt4d.y, pnt4d.z);
         Point3d uvt = findPoint (pnt);
         if (uvt == null) {
            return false;
         }
         double det = evalJacUVT (uvt, pnts).determinant ();
         if (det < 0) {
            return false;
         }
      }
      return true;
   }
   
   public NFFDPoint3d getNFFDPoint (Point3d pnt) {
      return myFFDPnts.get (pnt);
   }

   public SparseBlockMatrix createBasisMatrix (Point3d uvt) {
      if (myUVT == null) {
         throw new ImproperStateException ("NFFD Grid not initialized!");
      }

      int [] colB = new int [numCtrlPntsUVT[0] * numCtrlPntsUVT[1] * numCtrlPntsUVT[2]];
      int [] rowB = new int [1];
      rowB[0] = 3;
      for (int i = 0; i < colB.length; i++) {
         colB[i] = 3;
      }
      SparseBlockMatrix B = new SparseBlockMatrix (rowB, colB);
      Matrix3x3DiagBlock base;

      // initialize basis
      double [][] basis = new double [3][];
      for (int i = 0; i < 3; i++) {
         basis[i] = new double [degreesUVT[i] + 1];
      }
      // lower bound of knots span
      int [] t0 =  new int [3];
      // initialize degrees
      // evaluate basis and retrieve degrees
      for (int i = 0; i < 3; i++) {
         t0[i] = findSpan (uvt.get (i), KnotsUVT[i], degreesUVT[i]);
         evalBasis (basis[i], uvt.get (i), t0[i], KnotsUVT[i], degreesUVT[i]);
      }

      double w = 0;
      for (int i = 0; i <= degreesUVT[0]; i++) {
         int d_i = getCtrlIndex (t0[0], i, degreesUVT[0]);
         double B_i = basis[0][i];
         for (int j = 0; j <= degreesUVT[1]; j++) {
            int d_j = getCtrlIndex (t0[1], j, degreesUVT[1]);
            double B_j = basis[1][j];
            for (int k = 0; k <= degreesUVT[2]; k++) {
               int d_k = getCtrlIndex (t0[2], k, degreesUVT[2]);
               double B_k = basis[2][k];

               int idx = indexMap(d_i, d_j, d_k);
               Vector4d cpnt = myCtrlPnts.get( idx );
               double wbb = cpnt.w * B_i * B_j * B_k;
               base = new Matrix3x3DiagBlock (wbb, wbb, wbb);

               B.addBlock (0, idx, base);
               w += wbb;

            }
         }
      }
      B.scale (w);

      return B;
   }



   public boolean makeAndAppendBasis (SparseBlockMatrix Basis, Point3d uvt) {
      if (myUVT == null) {
         throw new ImproperStateException ("NFFD Grid not initialized!");
      }

      int num = numCtrlPntsUVT[0] * numCtrlPntsUVT[1] * numCtrlPntsUVT[2];

      Matrix3x3DiagBlock base;

      if (Basis.numBlockCols () < num) {
         int [] colB = new int [num-Basis.numBlockCols ()];
         for (int i = 0; i < colB.length; i++) {
            colB[i] = 3;
         }
         Basis.addCols (colB, colB.length);
      }
      else if (Basis.numBlockCols () > num) {
         throw new IllegalArgumentException ("Incompatible basis matrix");
      }
      Basis.addRow (3);

      // initialize basis
      double [][] basis = new double [3][];
      for (int i = 0; i < 3; i++) {
         basis[i] = new double [degreesUVT[i] + 1];
      }
      // lower bound of knots span
      int [] t0 =  new int [3];
      // initialize degrees
      // evaluate basis and retrieve degrees
      for (int i = 0; i < 3; i++) {
         t0[i] = findSpan (uvt.get (i), KnotsUVT[i], degreesUVT[i]);
         evalBasis (basis[i], uvt.get (i), t0[i], KnotsUVT[i], degreesUVT[i]);
      }

      double w = 0;
      for (int i = 0; i <= degreesUVT[0]; i++) {
         int d_i = getCtrlIndex (t0[0], i, degreesUVT[0]);
         double B_i = basis[0][i];
         for (int j = 0; j <= degreesUVT[1]; j++) {
            int d_j = getCtrlIndex (t0[1], j, degreesUVT[1]);
            double B_j = basis[1][j];
            for (int k = 0; k <= degreesUVT[2]; k++) {
               int d_k = getCtrlIndex (t0[2], k, degreesUVT[2]);
               double B_k = basis[2][k];

               int idx = indexMap(d_i, d_j, d_k);
               Vector4d cpnt = myCtrlPnts.get( idx );
               double wbb = cpnt.w * B_i * B_j * B_k;
               base = new Matrix3x3DiagBlock (wbb, wbb, wbb);

               Basis.addBlock (Basis.numBlockRows ()-1, idx, base);
               w += wbb;

            }
         }
      }

      base = (Matrix3x3DiagBlock)Basis.firstBlockInRow (Basis.numBlockRows ()-1);
      while (base != null) {
         base.scale (w);
         base = (Matrix3x3DiagBlock)base.next ();
      }

      return true;
   }

   //----------------------------------------- Mesh --------------------------------------//

   private HashMap<Point3d, NFFDPoint3d> makeFFDPointsMap (MeshBase mesh) {
      HashMap<Point3d, NFFDPoint3d> vtxMap = new HashMap<Point3d, NFFDPoint3d>();
      for (Vertex3d vtx: mesh.getVertices ()) {
         NFFDPoint3d ffdpnt = new NFFDPoint3d ();
         ffdpnt.setFFD (this);
         ffdpnt.setUndeformedPosition (vtx.getWorldPoint ());
         vtxMap.put (vtx.pnt, ffdpnt);
      }
      return vtxMap;
   }

   protected void embedMesh (MeshBase mesh) {
      myMeshes.add (mesh);
      HashMap<Point3d, NFFDPoint3d> map = makeFFDPointsMap(mesh);
      myMeshFFDPnts.put (mesh, map);
      myFFDPnts.putAll (map);
   }

   /**
    * set mesh without updating mesh 
    * FFD initial state
    * @param mesh
    */
   protected void embedMeshes(Collection<MeshBase> meshes) {
      for (MeshBase mesh : meshes) {
         embedMesh(mesh);
      }
   }

   public void removeMesh (MeshBase mesh) {
      if (!myMeshes.contains (mesh)) {
         throw new NullPointerException("Mesh not embedded in this FFD");
      }
      do {
         myMeshes.remove (mesh);
      }while(myMeshes.contains (mesh));
      myMeshFFDPnts.remove (mesh);
   }

   public void clearEmbeddedMeshes () {
      myMeshes.clear ();
      myMeshFFDPnts.clear ();
      myFFDPnts.clear ();
   }

   public MeshBase getMesh (int i) {
      if (i > myMeshes.size ()) {
         throw new IllegalArgumentException (
         "index out of bound");
      }
      return myMeshes.get (i);
   }

   public void getMeshes(ArrayList<MeshBase> embedMesh) {
      embedMesh.addAll (myMeshes);
   }

   public ArrayList<MeshBase> getMeshes () {
      return myMeshes;
   }

   public void advanceMeshInitialStates (MeshBase mesh) {
      if (!myMeshes.contains (mesh)) {
         throw new IllegalArgumentException (
         "Mesh not embedded");
      }
      HashMap<Point3d, NFFDPoint3d> map = myMeshFFDPnts.get (mesh);
      for (Vertex3d vtx : mesh.getVertices ()) {
         NFFDPoint3d ffdpnt = map.get (vtx.pnt);
         ffdpnt.setUndeformedPosition (vtx.getWorldPoint ());
         ffdpnt.evalUVT ();
      }
   }

   public void advanceMeshInitialStates (int i) {
      try {
         MeshBase mesh = getMesh(i);
         advanceMeshInitialStates (mesh);
      }
      catch (IllegalArgumentException e) {
         e.printStackTrace ();
         return;
      }
   }

   /**
    * set current mesh as initial state
    */
   public void advanceMeshInitialStates() {
      for (MeshBase mesh : myMeshes) {
         HashMap<Point3d, NFFDPoint3d> map = myMeshFFDPnts.get (mesh);
         for (Vertex3d vtx : mesh.getVertices ()) {
            NFFDPoint3d ffdpnt = map.get (vtx.pnt);
            ffdpnt.setUndeformedPosition (vtx.getWorldPoint ());
            ffdpnt.evalUVT ();
         }
      }
   }

   public void updateMesh (MeshBase mesh) {
      updateGridState ();
      for (Vertex3d vtx : mesh.getVertices ()) {
         Point3d pnt = new Point3d (vtx.getWorldPoint ());
         Point3d uvt = findPoint (pnt);
         eval (pnt, uvt);
         if (mesh.meshToWorldIsIdentity ()) {
            vtx.setPosition (pnt);
         }
         else {
            pnt.inverseTransform (mesh.getMeshToWorld ());
            vtx.setPosition (pnt);
         }
      }
      mesh.notifyVertexPositionsModified ();
   }

   public void updateMeshes () {
      updateGridState ();
      for (MeshBase mesh : myMeshes) {
         HashMap<Point3d, NFFDPoint3d> map = myMeshFFDPnts.get (mesh);
         for (Vertex3d vtx : mesh.getVertices ()) {
            NFFDPoint3d ffdpnt = map.get (vtx.pnt);
            Point3d newPnt = ffdpnt.computeFFD ();
            if (mesh.meshToWorldIsIdentity ()) {
               vtx.setPosition (newPnt);
            }
            else {
               newPnt.inverseTransform (mesh.getMeshToWorld ());
               vtx.setPosition (newPnt);
            }
         }
         mesh.notifyVertexPositionsModified ();
      }
   }

   public int [][] edgeVtxIdx;
   protected void makeEdgePointIndex() {
      ArrayList<int []> ins = new ArrayList<int []>();
      int [] in;
      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[1]; j++) {
            for (int k = 0; k < numCtrlPntsUVT[2]; k++) {
               if (i != numCtrlPntsUVT[0]-1) {
                  in = new int [2];
                  in [0] = indexMap (i, j, k);
                  in [1] = indexMap (i+1, j, k);
                  ins.add (in);

               }
               if (j != numCtrlPntsUVT[1]-1) {
                  in = new int [2];
                  in [0] = indexMap (i, j, k);
                  in [1] = indexMap (i, j+1, k);
                  ins.add (in);
               }
               if (k != numCtrlPntsUVT[2]-1) {
                  in = new int [2];
                  in [0] = indexMap (i, j, k);
                  in [1] = indexMap (i, j, k+1);
                  ins.add (in);
               }
            }
         }
      }
      edgeVtxIdx = new int [ins.size ()][2];
      for (int i = 0; i < ins.size (); i++) {
         edgeVtxIdx[i] = ins.get (i);
      }
   }





   boolean [] myCPMarks;
   boolean [] myCUMarks;
   int numSubCPs;
   int numSubCUs;

   AABBTree mySubAABBTree = null;
   boolean mySubBVTreeValid = false;

   int [] myMinIdx = new int [3];
   int [] myMaxIdx = new int [3];


   public void createSubdomain (int [] minIdx, int [] maxIdx) {
      for (int i = 0; i < 3; i++) {
         myMinIdx [i] = minIdx[i];
         myMaxIdx [i] = maxIdx[i];
      }

      // find units
      int num0 = maxIdx[0] - minIdx[0] + 1;
      int num1 = maxIdx[1] - minIdx[1] + 1;
      int num2 = maxIdx[2] - minIdx[2] + 1;
      numSubCPs = num0 * num1 * num2;

      myCPMarks = new boolean [numControlPoints ()];
      for (int i = minIdx[0]; i <= maxIdx[0]; i++) {
         for (int j = minIdx[1]; j <= maxIdx[1]; j++) {
            for (int k = minIdx[2]; k <= maxIdx[2]; k++) {
               myCPMarks [indexMap (i, j, k)] = true;
            }
         }
      }

      myCUMarks = new boolean [myUnits.size ()];
      int idx = 0;
      numSubCUs = 0;
      
      int [] minIdx1 = minIdx.clone ();
      int [] maxIdx1 = maxIdx.clone ();
      
      
      for (int i = 0; i < 3; i++) {
         minIdx1[i] = minIdx1[i] - degreesUVT[i] + 1;
         if (minIdx1[i] < 0) {
            minIdx1[i] = 0;
         }
         maxIdx1[i] = maxIdx1[i] + degreesUVT[i];
         if (maxIdx1[i] >= numCtrlPntsUVT[i]) {
            maxIdx1[i] = numCtrlPntsUVT[i]-1;
         }
      }

      for (NFFD3dControlUnit unit : myUnits) {
         boolean flag = true;
         int [] indices = unit.myVertexIndices;
         
         for (int i = 0; i < 3; i++) {
            if (indices[i] > maxIdx1[i] || indices[i] < minIdx1[i]) {
               flag = false;
               break;
            }
         }

         if (flag) {
            myCUMarks[idx] = true;
            numSubCUs ++;
         }
         idx++;
      }
      mySubBVTreeValid = false;
   }
   
   
   public boolean [] getSubdomainCPMarks () {
      if (myCPMarks == null) {
        return null;
      }
      return myCPMarks.clone ();
   }

   /**
    * uvt indices in sub-domain map to sub-domain 
    * control point index
    * @param uI
    * @param vI
    * @param tI
    * @return
    */
   public int subdomainIndexMap (int uI, int vI, int tI) {
      int num0 = myMaxIdx[0] - myMinIdx[0] + 1;
      int num1 = myMaxIdx[1] - myMinIdx[1] + 1;
      int num2 = myMaxIdx[2] - myMinIdx[2] + 1;

      if (uI < 0 || uI >= num0) {
         throw new IndexOutOfBoundsException (uI + ", " + num0);
      }
      if (vI < 0 || vI >= num1) {
         throw new IndexOutOfBoundsException (vI + ", " + num1);
      }
      if (tI < 0 || tI >= num2) {
         throw new IndexOutOfBoundsException (tI + ", " + num2);
      }

      int idx = uI * num1 * num2 + vI * num2 + tI;
      return idx;
   }
   
   /**
    * control point index map from full domain to 
    * sub-domain;
    * @param index in full domain
    * @return index in sub-domain
    */
   public int fullToSubdomainCPMap (int index) {
      
      // full domain uvt
      int [] idxUVT = inverseIndexMap (index);
      // sub-domain uvt
      for (int i = 0; i < 3; i++) {
         idxUVT[i] = idxUVT[i] - myMinIdx[i];
      }

      return subdomainIndexMap (idxUVT[0], idxUVT[1], idxUVT[2]);
   }
   
   /**
    * control point index in sub-domain maps to uvt indices 
    * of sub-domain
    * @param index
    * @return
    */
   public int [] inverseSubdomainIndexMap (int index) {
      int [] nums = numSubdomainUVTCPs();
      int numU = nums[0];
      int numV = nums[1];
      int numT = nums[2];
      
      int idxU = index / (numV * numT);
      int idxV = (index - idxU * numV * numT) / numT;
      int idxT = index - idxU * numV * numT - idxV * numT;
      
      if (idxU < 0 || idxU >= numU) {
         throw new IndexOutOfBoundsException ("");
      }
      if (idxV < 0 || idxV >= numV) {
         throw new IndexOutOfBoundsException ("");
      }
      if (idxT < 0 || idxT >= numT) {
         throw new IndexOutOfBoundsException ("");
      }
      
      int [] idxUVT = {idxU, idxV, idxT};
      
      return idxUVT;
   }
   
   public int numSubdomainCPs () {
      return numSubCPs;
   }
   
   public int [] numSubdomainUVTCPs () {
      int num0 = myMaxIdx[0] - myMinIdx[0] + 1;
      int num1 = myMaxIdx[1] - myMinIdx[1] + 1;
      int num2 = myMaxIdx[2] - myMinIdx[2] + 1;
      
      int [] nums = {num0, num1, num2};
      return nums;
   }
   

   public Point3d findPointInSubdomain (Point3d pntInWorld) {
      if (myUVT == null) {
         throw new ImproperStateException ("NFFD Grid not initialized!");
      }

      NFFD3dControlUnit unit = findSubContainingUnit(pntInWorld);

      if (unit == null) {
         System.out.println ("Control unit not found! [" + pntInWorld + "]");
         return null;
      }

      int [][] ranges = unit.getKnotSpanRange ();
      Point3d uvt = new Point3d ();
      for (int i = 0; i < 3; i++) {
         double lower = KnotsUVT[i][ranges[i][0]];
         double upper = KnotsUVT[i][ranges[i][1]];
         uvt.set (i, (lower + upper)/2.0);
      }

      if (findUVT (uvt, pntInWorld, uvt) < 0) {
         return null;
      }
      return uvt;
   }


   protected BVTree getSubBVTree() {
      if (mySubAABBTree == null || !mySubBVTreeValid) {
         mySubAABBTree = new AABBTree();
         mySubAABBTree.setMaxLeafElements (2);
         Boundable[] elements = new Boundable[numSubCUs];
         int idx = 0;
         for (int i = 0; i < myUnits.size (); i++) {
            if (!myCUMarks[i]) {
               continue;
            }
            elements[idx] = myUnits.get(i);
            idx++;
         }

         mySubAABBTree.build(elements, numSubCUs);
         mySubBVTreeValid = true;
      }
      return mySubAABBTree;
   }

   /**
    * Returns the control unit within the NFFD that contains a specified
    * point, or <code>null</code> if there is no such element.
    * 
    * @param pnt Point for which containing unit is desired.
    * @return containing unit, or null.
    */
   public NFFD3dControlUnit findSubContainingUnit(Point3d pnt) {
      BVTree bvtree = getSubBVTree();
      ArrayList<BVNode> nodes = new ArrayList<BVNode>();
      //System.out.println (pnt);
      bvtree.intersectPoint(nodes, pnt);
      //System.out.println ("num nodes " + nodes.size());
      if (nodes.size() == 0) {
         return null;
      }
      for (BVNode n : nodes) {
         Boundable[] elements = n.getElements();
         for (int i = 0; i < elements.length; i++) {
            ((NFFD3dControlUnit)elements[i]).generateConvexHull ();
            if (((NFFD3dControlUnit)elements[i]).isInsideConvexHull (pnt)) {
               return (NFFD3dControlUnit)elements[i];
            }
         }
      }
      return null;
   }
   
   public boolean isInsideSubdomain (Point3d uvt) {
      
      boolean inside = true;
      
      for (int i = 0; i < 3; i++) {
         double u = uvt.get (i);
         int ko = findSpan (u, KnotsUVT[i], degreesUVT[i]);
         int min = ko - degreesUVT[i];
         int max = ko;
         
         if (myMinIdx[i] > max || myMaxIdx[i] < min) {
            inside = false;
            break;
         }
      }
      
      return inside;
      
   }
   
   public boolean isSubdomainCtrlPnt (int idx) {
      return myCPMarks[idx];
   }


   public SparseBlockMatrix createSubBasisMatrix (Point3d uvt) {
      if (myUVT == null) {
         throw new ImproperStateException ("NFFD Grid not initialized!");
      }

      int [] colB = new int [numSubCPs];
      int [] rowB = new int [1];
      rowB[0] = 3;
      for (int i = 0; i < colB.length; i++) {
         colB[i] = 3;
      }
      SparseBlockMatrix B = new SparseBlockMatrix (rowB, colB);
      Matrix3x3DiagBlock base;

      // initialize basis
      double [][] basis = new double [3][];
      for (int i = 0; i < 3; i++) {
         basis[i] = new double [degreesUVT[i] + 1];
      }
      // lower bound of knots span
      int [] t0 =  new int [3];
      // initialize degrees
      // evaluate basis and retrieve degrees
      for (int i = 0; i < 3; i++) {
         t0[i] = findSpan (uvt.get (i), KnotsUVT[i], degreesUVT[i]);
         evalBasis (basis[i], uvt.get (i), t0[i], KnotsUVT[i], degreesUVT[i]);
      }

      double w = 0;
      for (int i = 0; i <= degreesUVT[0]; i++) {
         int d_i = getCtrlIndex (t0[0], i, degreesUVT[0]);
         double B_i = basis[0][i];
         for (int j = 0; j <= degreesUVT[1]; j++) {
            int d_j = getCtrlIndex (t0[1], j, degreesUVT[1]);
            double B_j = basis[1][j];
            for (int k = 0; k <= degreesUVT[2]; k++) {
               int d_k = getCtrlIndex (t0[2], k, degreesUVT[2]);
               double B_k = basis[2][k];

               int idx = indexMap (d_i, d_j, d_k);
               Vector4d cpnt = myCtrlPnts.get( idx );
               double wbb = cpnt.w * B_i * B_j * B_k;
               base = new Matrix3x3DiagBlock (wbb, wbb, wbb);
          
               if (d_i < myMinIdx[0] || d_i > myMaxIdx[0]) {
                  w += wbb;
                  continue;
               }
               if (d_j < myMinIdx[1] || d_j > myMaxIdx[1]) {
                  w += wbb;
                  continue;
               }
               if (d_k < myMinIdx[2] || d_k > myMaxIdx[2]) {
                  w += wbb;
                  continue;
               }
               
               if (! myCPMarks[idx]) {
                  System.err.println ("Interanl Error!!!!!");
               }

               d_i = d_i - myMinIdx[0];
               d_j = d_j - myMinIdx[1];
               d_k = d_k - myMinIdx[2];
               idx = subdomainIndexMap(d_i, d_j, d_k);
               B.addBlock (0, idx, base);
               w += wbb;
            }
         }
      }
      if (B.numBlocks () == 0) {
         System.err.println ("warning: zero basis!");
      }
      B.scale (w);

      return B;
   }
   





   //----------------------------------------- For Test------------------------------------//
   public static void main(String [] args) {
      // for testing
      double [] knots = NFFD3d.createUniformKnotsVector (3, 10);
      VectorNd vec = new VectorNd (knots);
      System.out.println (vec);

      NFFD3d ffd = new NFFD3d ();
      int [] degrees = {3, 3, 3};
      int [] num = {20, 18, 20};
      ffd.setDegrees (degrees);
      ffd.setCtrlPntsNum (num);

      ffd.embedMesh (generatePointCloud(1, 25));
      ffd.createControlGrid ();

      Matrix3d Jac = ffd.evalJac (new Point3d (0.25, 0, 0));
      System.out.println (Jac);
      //ffd.advanceMeshInitialStates ();

      //ffd.updateMeshes ();
   }

   private static PointMesh generatePointCloud (double length, int num) {
      PointMesh mesh = new PointMesh();
      double unit = length / (num-1);
      for (int i = 0; i < num; i++) {
         for (int j = 0; j < num; j++) {
            for (int k = 0; k < num; k++) {
               Point3d pnt3d = new Point3d (
                  i*unit, j*unit, k*unit);
               mesh.addVertex (pnt3d);
            }
         }
      }
      RigidTransform3d Xmw = new RigidTransform3d (
         length/2, length/2, length/2);
      mesh.inverseTransform (Xmw);

      return mesh;
   }

   
   private String myName;
   private int myNumber;
   private boolean isFixed = false;
   private boolean isMarked = false;
   private CompositeComponent myParent;
   
   NavpanelVisibility myNavpanelVisibility = NavpanelVisibility.VISIBLE;
   
   
   @Override
   public Property getProperty (String pathName) {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public PropertyInfoList getAllPropertyInfo () {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public Iterator<? extends HierarchyNode> getChildren () {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean hasChildren () {
      return false;
   }

   @Override
   public void write (PrintWriter writer, NumberFormat fmt, Object ref)
      throws IOException {
      throw new UnsupportedOperationException ("");
   }

   @Override
   public boolean isWritable () {
      return false;
   }

   @Override
   public String getName () {
      return myName;
   }

   @Override
   public void setName (String name) throws IllegalArgumentException {
      myName = new String (name);
   }

   @Override
   public int getNumber () {
      return myNumber;
   }

   @Override
   public void setNumber (int num) {
      myNumber = num;
   }

   @Override
   public CompositeComponent getParent () {
      return myParent;
   }

   @Override
   public void setParent (CompositeComponent parent) {
      if (parent instanceof NFFDController) {
         myParent =  parent;
      }
      else {
         throw new IllegalArgumentException (
            "Not my parent!");
      }
   }

   @Override
   public void connectToHierarchy () {
      
   }

   @Override
   public void disconnectFromHierarchy () {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean isSelected () {
      // TODO Auto-generated method stub
      return false;
   }

   @Override
   public void setSelected (boolean selected) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean isMarked () {
      return isMarked;
   }

   @Override
   public void setMarked (boolean marked) {
      isMarked = marked;
   }

   @Override
   public boolean isFixed () {
      return isFixed;
   }

   @Override
   public void setFixed (boolean fixed) {
      isFixed = fixed;
   }

   @Override
   public NavpanelVisibility getNavpanelVisibility () {
      return myNavpanelVisibility;
   }

   @Override
   public void notifyParentOfChange (ComponentChangeEvent e) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void getHardReferences (List<ModelComponent> refs) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void getSoftReferences (List<ModelComponent> refs) {
      // TODO Auto-generated method stub
      
   }

   @Override
   public boolean hasState () {
      return false;
   }

   @Override
   public void scan (ReaderTokenizer rtok, Object ref) throws IOException {
      // TODO Auto-generated method stub
      
   }

   @Override
   public void postscan (Deque<ScanToken> tokens, CompositeComponent ancestor)
      throws IOException {
      // TODO Auto-generated method stub
   }

   @Override
   public void updateReferences (boolean undo, Deque<Object> undoInfo) {
      // TODO Auto-generated method stub
      
   }
}
