package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import artisynth.core.mechmodels.Point;
import artisynth.models.modelOrderReduction.Matrix3x3DiagBlock;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.transformers.NFFD3d.ControlGridMethod;
import artisynth.models.swallowingRegistrationTool.utilities.MeshConvexHull;
import maspack.geometry.MeshBase;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.DrawMode;

public class ReducedNFFD3d extends NFFD3d{
   
   protected boolean [] myControlPointMarks;
   List<Vector4d> myReducedCtrlPnts = new ArrayList<Vector4d> ();
   
   private Set<Point3d> slavePnts = new HashSet<Point3d> ();
   int numCP = 0;
   
   public ReducedNFFD3d () {
      super ();
   }
   
   public ReducedNFFD3d (int [] numCtrlPnts, int [] degrees) {
      super (numCtrlPnts, degrees);
   }

   public ReducedNFFD3d (int [] numCtrlPnts, int [] degrees, 
      MeshBase meshes, Set<Point3d> slaves) {
      super (numCtrlPnts, degrees);
      embedMesh (meshes);
      createControlGrid ();
      advanceMeshInitialStates ();
      slavePnts.addAll (slaves);
   }
   
   public void build (MeshBase mesh, Set<Point3d> slaves) {
      embedMesh (mesh);
      slavePnts.addAll (slaves);
      createControlGrid ();
      advanceMeshInitialStates ();
   }
   
   public void build (Set<MeshBase> meshes, Set<Point3d> slaves) {
      embedMeshes (meshes);
      slavePnts.addAll (slaves);
      createControlGrid ();
      advanceMeshInitialStates ();
   }
   
   public void build (int [] numCtrlPnts, int [] degrees, 
      MeshBase meshes, Set<Point3d> slaves) {
      setDegrees (degrees);
      setCtrlPntsNum (numCtrlPnts);
      embedMesh (meshes);
      slavePnts.addAll (slaves);
      createControlGrid ();
      advanceMeshInitialStates ();
   }
   
   public void build (int [] numCtrlPnts, int [] degrees, 
      Set<MeshBase> meshes, Set<Point3d> slaves) {
      setDegrees (degrees);
      setCtrlPntsNum (numCtrlPnts);
      embedMeshes (meshes);
      slavePnts.addAll (slaves);
      createControlGrid ();
      advanceMeshInitialStates ();
   }
   
   /**
    * only use for reduce NFFD
    * @param pnt
    */
   public void addSlavePnts (Point3d pnt) {
      slavePnts.add (new Point3d (pnt));
   }
   
   public boolean containSlavePnts (Point3d pnt) {
      return slavePnts.contains (pnt);
   }
   
   public void clearSlavePnts () {
      slavePnts.clear ();
   }
   
   @Override
   public  void buildControlUnitList () {
      super.buildControlUnitList ();

      // mark points inside the convex hull
      boolean [] marks = new boolean [numControlPoints ()];
      int num = 0;

      // units need to be removed
      List<NFFD3dControlUnit> units = new ArrayList <NFFD3dControlUnit> ();
      units.addAll (super.myUnits);
      
      // find occupied nodes
      for (Point3d pnt : slavePnts) {
         NFFD3dControlUnit unit = super.
         findContainingUnit (pnt);
         units.remove (unit);
         int [] ids = unit.myVertexIndices;
         for (int idx : ids) {
            if (!marks[idx]) {
               num++;
            }
            marks [idx] = true;
         }
      }

      // build convex hull
      Point3d [] ons = new Point3d [num];
      int idx = 0;
      for (int i = 0; i < marks.length; i++) {
         if (marks[i]) {
            Vector4d cp  = super.getControlPoint (i);
            ons[idx++] = new Point3d  (cp.x, cp.y, cp.z);
         }
      }
      MeshConvexHull hull = new MeshConvexHull ();
      hull.build (ons);
      
      boolean [] visited = marks.clone ();
      // remove units
      boolean [] unitsMarks = new boolean [units.size ()];
      for (int i = 0; i < units.size (); i++) {
         NFFD3dControlUnit unit = units.get (i);
         int [] ids = unit.myVertexIndices;
         for (int id : ids) {
            if (visited[id]) {
               if (marks[id]) {
                  unitsMarks[i] = true;
                  break;
               }
            }
            else {
               visited [id] = true;
               Vector4d cp = getControlPoint (id);
               Point3d pnt = new Point3d (cp.x, cp.y, cp.z);
               if (hull.isInsideConvexHull (pnt)) {
                  marks [id] = true;
                  // is inside the convex hull
                  unitsMarks [i] = true;
                  break;
               }
            }
         }
      }
      
      for (int i = 0; i < unitsMarks.length; i++) {
         if (unitsMarks[i]) {
            units.remove (units.get (i));
         }
      }
      
      for (int i = 0; i < units.size (); i++) {
         NFFD3dControlUnit unit = units.get (i);
         myUnits.remove (unit);
      }
      
      
      myControlPointMarks = new boolean [numControlPoints ()];
      myReducedCtrlPnts.clear ();
      for (NFFD3dControlUnit unit : myUnits) {
         int [] ids = unit.myVertexIndices;
         for (int id : ids) {
            myControlPointMarks[id] = true;
         }
      }

      idx = 0;
      for (Boolean mark: myControlPointMarks) {
         if (mark) myReducedCtrlPnts.add (
            myCtrlPnts.get (idx++));
      }
      
      slavePnts.clear ();
      numCP = countReducedCtlPnts ();
   }
   
   private int countReducedCtlPnts () {
      int num = 0;
      for (Boolean mark: myControlPointMarks) {
         if (mark) num++;
      }
      return num;
   }
   
   public int numReducedCtlPnts () {

      return numCP;
   }
   
   public boolean isActivePointIndex (int idx) {
      if (idx >= numControlPoints() || idx < 0) {
         throw new IllegalArgumentException (
            "out of range");
      }
      
      return myControlPointMarks[idx];
   }
   
   @Override
   public SparseBlockMatrix createBasisMatrix (Point3d uvt) {
      if (myUVT == null) {
         throw new ImproperStateException ("NFFD Grid not initialized!");
      }

      int [] colB = new int [numReducedCtlPnts ()];
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
      // TODO: not detected yet
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
               
               B.addBlock (0, myReducedCtrlPnts.
                  indexOf (cpnt), base);
               w += wbb;
            }
         }
      }
      B.scale (w);

      return B;
   }
   

   /**
    * {@inheritDoc}
    */
   public void render (Renderer renderer, RenderProps props, int flags) {
      boolean selecting = renderer.isSelecting();
      
      if (numControlPoints() == 0) {
         return;
      }    

      // draw the control polygon
      if (!selecting) {
         renderer.setColor (edgeColor);
      }
      else {
         renderer.setHighlighting (true);
      }
      
      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[1]; j++) { // pnt.setFromHomogeneous

            renderer.beginDraw (DrawMode.LINE_STRIP);
            for (int k = 0; k < numCtrlPntsUVT[2]; k++) {
               int idx = indexMap(i, j, k);
               if (myControlPointMarks[idx]) {
                  Vector4d cpnt = myCtrlPnts.get(idx);
                  renderer.addVertex (cpnt.x, cpnt.y, cpnt.z);
                  if (!myControlPointMarks[idx+1]) {
                     break;
                  }
               }
            }
            renderer.endDraw ();
         }
      }

      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[2]; j++) { // pnt.setFromHomogeneous


            renderer.beginDraw (DrawMode.LINE_STRIP);

            for (int k = 0; k < numCtrlPntsUVT[1]; k++) {
               int idx = indexMap(i, k, j);
               if (myControlPointMarks[idx]) {
                  Vector4d cpnt = myCtrlPnts.get(idx);
                  renderer.addVertex (cpnt.x, cpnt.y, cpnt.z);
                  if (!myControlPointMarks[idx+1]) {
                     break;
                  }
               }
               
            }
            renderer.endDraw ();
         }
      }

      for (int i = 0; i < numCtrlPntsUVT[1]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[2]; j++) { // pnt.setFromHomogeneous

            renderer.beginDraw (DrawMode.LINE_STRIP);

            for (int k = 0; k < numCtrlPntsUVT[0]; k++) {
               int idx = indexMap(k, i, j);
               if (myControlPointMarks[idx]) {
                  Vector4d cpnt = myCtrlPnts.get(idx);
                  renderer.addVertex (cpnt.x, cpnt.y, cpnt.z);
                  if (!myControlPointMarks[idx+1]) {
                     break;
                  }
               }
            }
            renderer.endDraw ();
         }
      }

      renderer.setHighlighting (false);
   }
   
   @Override
   public void setCtrlPntPosition (int idx, Point3d pos) {
      if (idx >= myReducedCtrlPnts.size () || idx < 0) {
         throw new IllegalArgumentException ("Index is outof range");
      }
      
      int idx1 = myCtrlPnts.indexOf (myReducedCtrlPnts.get (idx));
      Point cp = myCtrlPntAgents.get (idx1);
      cp.setPosition (pos);
      myBVTreeValid = false;
   }
   
   @Override
   public void setCtrlPntPosition (int uIdx, int vIdx, int tIdx, Point3d pos) {
      if (uIdx >= numCtrlPntsUVT[0] || vIdx >= numCtrlPntsUVT[1] || 
      tIdx >= numCtrlPntsUVT[2]) {
         throw new IllegalArgumentException ("Index is out of range");
      }
      
      int idx1 = myReducedCtrlPnts.indexOf (myCtrlPnts.
         get (indexMap (uIdx, vIdx, tIdx)));
      
      setCtrlPntPosition (idx1, pos);
   }
   
   @Override
   public void setCtrlPntPositions (MatrixNd P) {
      if (P.colSize () != 3) {
         throw new ImproperSizeException("Incompatible size");
      }
      if (P.rowSize () != myReducedCtrlPnts.size ()) {
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
   
   @Override
   public void setCtrlPntPositions (VectorNd P) {
      if (P.size() != (3*myReducedCtrlPnts.size ())) {
         throw new ImproperSizeException("Incompatible size");
      }
      Point3d pos= new Point3d();

      for (int i = 0; i < myReducedCtrlPnts.size (); i++) {
         pos.x = P.get (i*3);
         pos.y = P.get (i*3+1);
         pos.z = P.get (i*3+2);
         setCtrlPntPosition (i, pos);
      }
   }
   
   
   public Vector4d[] getReducedControlPoints() {
      return myReducedCtrlPnts.toArray (new Vector4d[0]);
   }
}
