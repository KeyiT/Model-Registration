package artisynth.models.swallowingRegistrationTool.transformers;

import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;

import javax.swing.JMenuItem;

import artisynth.core.driver.Main;
import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableModelBase;
import artisynth.core.modelbase.StepAdjustment;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.utilities.MeshConvexHull;
import artisynth.models.swallowingRegistrationTool.utilities.PCA;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVTree;
import maspack.geometry.MeshBase;
import maspack.geometry.OBBTree;
import maspack.geometry.PointMesh;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.PointStyle;

public class BezierFFD3d extends RenderableModelBase implements ActionListener{

   public int NList[]; // required input
   protected MatrixNd myP; // N x D, set according to mesh 
   private Vector3d myS; // set according to FFDBBVs
   private Vector3d myT; // set according to FFDBBVs
   private Vector3d myU; // set according to FFDBBVs
   protected BernsteinPolynomia myBPoly; // set according to myP, NList

   // embedded meshes
   protected ArrayList <MeshBase> myMeshes = new ArrayList<MeshBase> ();
   private HashMap<MeshBase, HashMap<Point3d, FFDPoint3d>> myMeshFFDPnts = new HashMap<MeshBase, HashMap<Point3d, FFDPoint3d>> ();

   // cage
   protected RigidTransform3d myX1W; // set according to myMesh(PCA) 
   protected RigidTransform3d myR1W;
   public int [] interialPlaneNum = new int [3]; // set according to NList
   protected PointList<ControlPoint> controlPntList;
   protected ControlPoint [][][] controlPnts;
   public Point3d [] FFDBBVs = new Point3d[8]; // set according to myMesh
   public int [][] edgeVtxIdx;
   public int [][][][] ijkdMap;
   public int [][][] ijkMap;

   public Matrix3d myJacobian = new Matrix3d();

   private static String DEFAULT_NAME = "BezierFFD";
   protected RenderProps controlPointsRenderProps;





   /**
    * define control points
    * @author KeyiTang
    *
    */
   public class ControlPoint extends FFDPoint {

      protected int myIndex;
      protected int [] myLatticalIndex = new int [3];


      public ControlPoint () {
         super();
      }

      protected void setIndex (int argInd) {
         myIndex = argInd;
      }

      protected int getIndex () {
         return myIndex;
      }

      /**
       * @return myLatticalIndex
       */
      protected int [] getLatticalIndex () {
         return myLatticalIndex;
      }

      /**
       * @param argInd must be length-3 integer array
       * argInd: [0, myInterialPlaneNum]
       */
      protected void setLatticalIndex (int [] argInd) {
         myLatticalIndex = argInd;
      }

      /**
       * @param argIndX
       * @param argIndY
       * @param argIndZ
       * argInd: [0, myInterialPlaneNum]
       */
      protected void setLatticalIndex (int argIndX, int argIndY, int argIndZ) {
         int [] temInd = {argIndX, argIndY, argIndZ};
         setLatticalIndex (temInd);
      }

   }

   protected class FFDPoint extends Point{
      protected Point3d mystu = new Point3d();
      protected BezierFFD3d myFFD;

      public FFDPoint () {
         super ();
      }

      public FFDPoint (Point3d pnt) {
         super(pnt);
      }

      /**
       * set position in natural coordinate
       * @param argS
       * @param argT
       * @param argU
       */
      protected void setNaturalCoor(double argS, double argT, double argU) {
         mystu.set (argS, argT, argU);
      }

      /**
       * get position in natural coordinate
       * @return
       */
      public Point3d getNaturalCoor() {
         return mystu;
      }

      /**
       * compute undeformed position based on its
       * natural coordinate; 
       * X = Xo + s S + t T + u U
       * @return X undeformed position
       */
      public Point3d getUndeformedPosition () {
         if (mystu == null) {
            throw new ImproperStateException("natural coordinate not initialized");
         }
         if (myFFD == null) {
            throw new ImproperStateException("FFD not specified");
         }
         Point3d restPos = new Point3d();
         Matrix3d STU = myFFD.getSTUMat ();
         restPos.mul (STU, mystu);
         restPos.add (myFFD.getXo ());
         return restPos;
      }

      public BezierFFD3d getFFD() {
         return myFFD;
      }

      public void setFFD(BezierFFD3d ffd) {
         myFFD = ffd;
      }
   }


   // -------------------------------------
   // control point end
   // -------------------------------------

   public BezierFFD3d () {
      setNList (1, 1, 1);
      setName(DEFAULT_NAME);
   }

   // non-automatic grid generate method
   public BezierFFD3d (int [] argNList, Point3d [] bbvs) {
      setNList (argNList);
      setName(DEFAULT_NAME);
      regenerateFFDGrid (bbvs);
   }

   // automatic grid generate method
   public BezierFFD3d (int [] argNList, Collection<MeshBase> meshes) {
      setNList(argNList);
      embedMeshes(meshes);
      setName(DEFAULT_NAME);
      regenerateFFDGrid();
   }

   // automatic grid generate method
   public BezierFFD3d (int argNX, int argNY, int argNZ, Collection<MeshBase> meshes) {
      setNList(argNX, argNY, argNZ);
      embedMeshes(meshes);
      setName(DEFAULT_NAME);
      regenerateFFDGrid();
   }

   // -------------------------------------
   // mesh
   // -------------------------------------

   private HashMap<Point3d, FFDPoint3d> makeFFDPointsMap (MeshBase mesh) {
      HashMap<Point3d, FFDPoint3d> vtxMap = new HashMap<Point3d, FFDPoint3d>();
      for (Vertex3d vtx: mesh.getVertices ()) {
         FFDPoint3d ffdpnt = new FFDPoint3d ();
         ffdpnt.setFFD (this);
         ffdpnt.setUndeformedPosition (vtx.getWorldPoint ());
         vtxMap.put (vtx.pnt, ffdpnt);
      }
      return vtxMap;
   }

   public void embedMesh (MeshBase mesh) {
      myMeshes.add (mesh);
      myMeshFFDPnts.put (mesh, makeFFDPointsMap(mesh));
   }

   /**
    * set mesh without updating mesh 
    * FFD initial state
    * @param mesh
    */
   public void embedMeshes(Collection<MeshBase> meshes) {
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
   }

   public void getMeshes(ArrayList<MeshBase> embedMesh) {
      embedMesh.addAll (myMeshes);
   }

   public ArrayList<MeshBase> getMeshes () {
      return myMeshes;
   }

   /**
    * set current mesh as initial state
    */
   public void advanceMeshInitialStates() {
      for (MeshBase mesh : myMeshes) {
         HashMap<Point3d, FFDPoint3d> map = myMeshFFDPnts.get (mesh);
         for (Vertex3d vtx : mesh.getVertices ()) {
            FFDPoint3d ffdpnt = map.get (vtx.pnt);
            ffdpnt.setUndeformedPosition (vtx.getWorldPoint ());
            ffdpnt.recomputeNaturalCoor ();
         }
      }
      // TODO: may remove later
      advanceMeshInitialState();
   }


   // -------------------------------------
   // Generate FFD Grid
   // -------------------------------------
   public void regenerateFFDGrid (Point3d [] bbvs) {
      setBBVs (bbvs);
      setSTU();
      computeX2STUJacobian();
      setControlPoints();
      setBernsteinPolynomia ();
      advanceMeshInitialStates();
   }

   public void regenerateFFDGridForSlave(MeshBase slave) {
      setBBVs(slave);
      setSTU();
      computeX2STUJacobian();
      setControlPoints();
      setBernsteinPolynomia ();
      advanceMeshInitialStates();
   }

   public void regenerateFFDGrid() {
      setBBVs();
      setSTU();
      computeX2STUJacobian();
      setControlPoints();
      setBernsteinPolynomia ();
      advanceMeshInitialStates();
   }

   // -------------------------------------
   // reset FFD
   // -------------------------------------

   /**
    * reset control points and update P 
    * matrix.
    */
   public void resetFFD() {
      resetControlPoints();
      updateP();
   }


   // -------------------------------------
   // properties
   // ------------------------------------


   // -------------------------------------
   // bounding box of FFD
   // -------------------------------------

   /**
    * get bounding box of mesh according to it's PCA
    * @param argMesh input mesh
    * @param resBBV bounding box 8 vertexes
    */
   //TODO
   public void computeBB (RigidTransform3d Xmw, MeshBase argMesh, Point3d [] resBBV) {
      if (Xmw != null) {
         Xmw.set (PCA.computeBBVs (argMesh, resBBV, null));
      }
   }

   /**
    * get bounding box of mesh according to it's PCA
    * @param argMesh input mesh
    * @param resBBV bounding box 8 vertexes
    */
   public static RigidTransform3d computeBBVs (MeshBase argMesh, Point3d [] resBBV) {
      // get covariance and the centroid of the mesh
      Matrix3d temCov = new Matrix3d();
      Vector3d temMeshCen = new Vector3d();
      computePCA (temCov, temMeshCen, argMesh);

      // get Mesh R1W
      SVDecomposition3d temSVD = new SVDecomposition3d();
      temSVD.factor (temCov);
      RigidTransform3d temX1W = new RigidTransform3d();
      temX1W.R.set (temSVD.getU ());

      /*
      if (temX1W.R.determinant() < 0) {
         // flip x axis to correct
         temX1W.R.m00 = -temX1W.R.m00;
         temX1W.R.m10 = -temX1W.R.m10;
         temX1W.R.m20 = -temX1W.R.m20;
      }*/

      // get Mesh Bounding Box
      Point3d pnt = new Point3d();
      double huge = Double.MAX_VALUE;
      Vector3d temMax = new Vector3d (-huge, -huge, -huge);
      Vector3d temMin = new Vector3d ( huge,  huge,  huge);
      for (int i=0; i<argMesh.numVertices(); i++) {
         pnt.inverseTransform (temX1W.R, argMesh.getVertices().get(i).pnt);
         pnt.updateBounds (temMin, temMax);
      }


      Vector3d diaVec = new Vector3d();
      diaVec.sub (temMax, temMin);
      diaVec.scale (0.015);
      temMax.add (diaVec);
      temMin.sub (diaVec);


      // get centroid of the mesh
      temMeshCen.add (temMax, temMin);
      temMeshCen.scale (0.5);
      temX1W.p.set (temMeshCen);



      // get Mesh Bounding Box 8 Vertexes
      // x is the highest order position
      // z is the lowest order position
      Vector3d [] temBBV = new Vector3d [2];
      temBBV[0] = temMin;
      temBBV[1] = temMax;
      int g = 0;
      Vector3d temVec = new Vector3d();
      for (int i = 0; i <=1 ; i++) {
         for (int j = 0; j <= 1; j++) {
            for (int k = 0; k <= 1; k++) {
               temVec.set (temBBV[i].x, temBBV[j].y, temBBV[k].z);
               temVec.transform (temX1W.R);
               resBBV[g] = new Point3d();
               resBBV[g].set (temVec);
               g++;
            }
         }
      }

      // get centroid of the mesh
      temX1W.p.set (temMeshCen);

      return temX1W;
   }

   /**
    * Based on "Non-rigid Transformations for Musculoskeletal Model", by Petr
    * Kellnhofer.
    */
   public static double computePCA (
      Matrix3d J, Vector3d c, MeshBase mesh) {

      Vector3d d = new Vector3d();
      double maxdSqr = 0;

      mesh.computeCentroid (c);

      J.setZero();
      int numv = mesh.numVertices();
      for (int i=0; i<numv; i++) {
         Vertex3d vtx = mesh.getVertices().get(i);
         d.sub (vtx.pnt, c);
         J.addOuterProduct (d, d);
         double dsqr = d.normSquared();
         if (dsqr > maxdSqr) {
            maxdSqr = dsqr;
         }
      }
      J.scale (1/(double)numv);
      return Math.sqrt (maxdSqr);
   }


   /**
    * set FFD bounding box according to PCA size of myMesh
    */
   public void setBBVs () {
      if (myMeshes.size () != 0) {
         PointMesh pm = new PointMesh();
         for (MeshBase mesh : myMeshes) {
            for (Vertex3d vtx : mesh.getVertices ()) {
               pm.addVertex (new Point3d (vtx.getWorldPoint ()));
            }
         }
         setBBVs(pm);
      }
      // TODO: may remove
      // for old method
      else {
         if (myMesh == null) {
            throw new ImproperStateException ("No embedded mesh!");
         }
         setBBVs(myMesh);
      }
   }

   /**
    * set FFD bounding box according to PCA size of input mesh
    * @param argMesh
    */
   public void setBBVs (MeshBase argMesh) {
      myX1W = PCA.computeBBVs (argMesh, FFDBBVs, null);
   }

   // TODO: need specify
   /**
    * set bounding box vertices; 
    *
    * @param bbvs 8 vertices bounding box of 
    */
   public void setBBVs (Point3d [] bbvs) {
      if (bbvs.length != 8) {
         throw new ImproperSizeException ("Incompatible bounding box vertices!");
      }

      int idx = 0;
      for (int i = 0; i < FFDBBVs.length; i++) {
         if (FFDBBVs[i] == null) {
            FFDBBVs[i] = new Point3d ();
         }
      }
      for (Point3d ver : FFDBBVs) {
         ver.set (bbvs[idx++]);
      }
   }


   /**
    * get 8 FFD bounding box vertexes
    * @return Point3d array
    */
   public Point3d [] getBBVs () {
      Point3d [] bbvs = new Point3d [8];
      int i = 0;
      for (Point3d bbv : FFDBBVs) {
         bbvs[i] = new Point3d (bbv);
         i++;
      }
      return bbvs;
   }

   /**
    * get single bounding box vertex 
    * @param argIndex
    * @return
    */
   public Point3d getBBV(int argIndex) {
      if (argIndex < 0 || argIndex > 7) {
         throw new ImproperSizeException (
         "Index out of feasible range");
      }
      Point3d bv = new Point3d ();
      bv.set (FFDBBVs[argIndex]);
      return bv;
   }

   /**
    * get single bounding box vertex
    * @param argIndX
    * @param argIndY
    * @param argIndZ
    * @return
    */
   public Point3d getBBV(int argIndX, int argIndY, int argIndZ) {
      String tem;
      tem = Integer.toString (argIndX);
      tem = tem + Integer.toString (argIndY);
      tem = tem + Integer.toString (argIndZ);
      //System.out.println ("getBBV: tem is : " + Integer.valueOf (tem, 2));
      Point3d bv = new Point3d ();
      bv.set (getBBV(Integer.valueOf (tem, 2)));
      return bv;
   }

   public Point3d getXo() {
      Point3d bv = new Point3d ();
      bv.set (getBBV(0));
      return getBBV(0);
   }
   // -------------------------------------
   // Set Control Point 
   // -------------------------------------
   /**
    * set DOFs of Bersein Polynomial in each direction
    * which are l, m, n in paper: Free-Form Deformation of Solid Geometric Models.
    * Input must greater or equal to 1
    * @param argNList has length: 3
    * when input length smaller than 3
    * Dof in each direction set to 1
    */
   public void setNList(int [] argNList) {
      if (argNList.length < 3) {
         setNList(1, 1, 1);
      } 
      else {
         setNList(argNList[0], argNList[1], argNList[2]);
      }
   }

   /**
    * set DOFs of Bernstein Polynomial in each direction
    * which are l, m, n in paper: Free-Form Deformation of Solid Geometric Models.
    * Input must greater or equal to 1
    * @param argXN
    * @param argYN
    * @param argZN
    */
   public void setNList(int argXN, int argYN, int argZN) {
      NList = new int [3];
      NList[0] = argXN;
      NList[1] = argYN;
      NList[2] = argZN;
      initializeControlPointInfo();
   }

   /**
    * get DOFs of Bernstein Polynomial in each direction
    * @return
    */
   public int [] getNList() {
      int [] nlist = new int [3];
      for (int i = 0; i < 3; i++) {
         nlist[i] = NList[i];
      }
      return nlist;
   }


   /**
    * get the control point number
    * @return 
    */
   public int getCtPntNum() {
      // the number of control points
      int totalN = 1;
      for (int temN : NList) {
         totalN = (temN+1) * totalN;
      }
      return totalN;
   }

   /**
    * set FFD local Frame
    * S -> X, T -> Y, U -> Z
    */
   protected void setSTU() {
      Point3d temXo = getBBV(0);
      myS = new Vector3d();
      myT = new Vector3d();
      myU = new Vector3d();
      myS.sub (getBBV(1, 0, 0),  temXo);
      myT.sub (getBBV(0, 1, 0),  temXo);
      myU.sub (getBBV(0, 0, 1),  temXo);
      //System.out.printf ("\nsetSTU: S is %f, %f, %f", myS.x, myS.y, myS.z);
      //System.out.printf ("\nsetSTU: T is %f, %f, %f", myT.x, myT.y, myT.z);
      //System.out.printf ("\nsetSTU: U is %f, %f, %f\n", myU.x, myU.y, myU.z);
   }

   /**
    * [S, T, U]
    * @return S T U matrix
    */
   public Matrix3d getSTUMat() {
      Matrix3d temMat = new Matrix3d();
      temMat.setColumn (0, myS);
      temMat.setColumn (1, myT);
      temMat.setColumn (2, myU);

      return temMat;
   }

   private void initializeControlPointInfo() {
      // the number of control points
      int totalN = getCtPntNum();
      controlPntList = new PointList<ControlPoint>(ControlPoint.class);
      controlPntList.setName ("control points");
      controlPnts = new ControlPoint [NList[0]+1][NList[1]+1][NList[2]+1];
      myP = new MatrixNd (totalN, 3);
   }

   /**
    * set control points and P matrix
    * and make edge
    */
   protected void setControlPoints () {
      controlPntList.clear ();
      removeAll ();
      // set control point
      int temInd = 0;
      Vector3d temVec = new Vector3d();
      Matrix3d temMat = new Matrix3d();
      for (int i=0; i <= NList[0] ; i ++) {
         for (int j = 0; j <= NList[1]; j++) {
            for (int k = 0; k <= NList[2]; k++) {
               //System.out.println("setControlPoints: temInd is " + temInd);
               Point3d Xo = new Point3d(getXo());
               ControlPoint ctPnt = new ControlPoint();
               controlPntList.add (ctPnt);
               controlPnts[i][j][k] = ctPnt;
               ctPnt.setFFD (this);
               ctPnt.setIndex (temInd);
               ctPnt.setLatticalIndex (i, j, k);
               ctPnt.setNaturalCoor ((double)i/(double)NList[0], (double)j/(double)NList[1], (double)k/(double)NList[2]);
               // set position of the control point
               ctPnt.setPosition (ctPnt.getUndeformedPosition ());
               // set P matrix
               myP.setRow (temInd, ctPnt.getUndeformedPosition ());         
               temInd ++;
            }
         }
      }
      makeEdge();
      setControlPointsRenderProps();
      addFixed(controlPntList);
   }


   /**
    * reset control points
    */
   public void resetControlPoints () {
      // set control point
      for (ControlPoint cp : controlPntList) {
         cp.setPosition (cp.getUndeformedPosition ());
      }
      setControlPointsRenderProps();
   }

   public PointList<ControlPoint> getControlPoints () {
      return controlPntList;
   }

   public ControlPoint getControlPoint(int idx) {
      return controlPntList.get (idx);
   }

   public ControlPoint getControlPoint (int i, int j, int k) {
      return controlPnts[i][j][k];
   }

   /**
    * rowSize of P is the number of control points
    * colSize of P is 3
    * @param P
    */
   public void setControlPointPositions(MatrixNd P) {
      if (P.colSize () != 3) {
         throw new ImproperSizeException("Incompatible size");
      }
      if (P.rowSize () != getCtPntNum ()) {
         throw new ImproperSizeException ("Incompatible size");
      }
      Point3d pnt3d = new Point3d();

      for (int i = 0; i < P.rowSize (); i++) {
         pnt3d.x = P.get (i, 0);
         pnt3d.y = P.get (i, 1);
         pnt3d.z = P.get (i, 2);
         controlPntList.get (i).setPosition (pnt3d);
      }

      updateP();
   }



   // -------------------------------------
   // Set Bernstein Polynomial
   // -------------------------------------

   /**
    * set Bernstein Polynomial
    */
   protected void setBernsteinPolynomia () {
      myBPoly = new BernsteinPolynomia(NList);
      //System.out.print("\n setBernsteinPolynomia P is: \n");
      myBPoly.setP (myP);
   }

   /**
    * return P vector
    */
   public VectorNd getPVec() {
      return myBPoly.getPVec ();
   }

   /**
    * return P Matrix
    */
   public MatrixNd getP() {
      MatrixNd p = new MatrixNd ();
      p.set (myP);
      return p;
   }

   /**
    * 
    * @param argX position in world position
    * @return bernstein basis
    */
   public VectorNd getB(Vector3d argX) {
      return myBPoly.makeBlendingFunctions (car2stu(new Vector3d(argX)));
   }

   /**
    * 
    * @param argX
    * @return
    */
   public MatrixNd getBMat(Vector3d argX) {
      return myBPoly.makeBlendingMatrix (new VectorNd(car2stu(new Vector3d(argX))));
   }

   public VectorNd getB() {
      return myBPoly.getB ();
   }

   // -------------------------------------
   // update control points
   // -------------------------------------

   /**
    * update P according to control Points
    * update P in BernsteinPolynomia
    */
   public final static void updateP(PointList<ControlPoint> argCtPnt, MatrixNd argP, BernsteinPolynomia argPoly) {
      for (int i =0; i < argCtPnt.size (); i++) {
         // set P matrix
         argP.setRow (i, argCtPnt.get (i).getPosition ());
      }
      argPoly.setP (argP);
   }

   /**
    * update P according to control Points
    * update P in BernsteinPolynomia
    */
   public void updateP() {
      updateP(controlPntList, myP, myBPoly);
   }

   // -------------------------------------
   // make FFD
   // -------------------------------------

   /**
    * from natural space to material space
    * @param argX point in natural space
    * @return s t u
    */
   public Point3d car2stu(Vector3d argX) {
      return car2stu(argX, myS, myT, myU, getXo ());
   }

   /**
    * from natural space to material space
    * @param argX point in natural space
    * @return s t u
    */
   public static Point3d car2stu(Vector3d argX, Vector3d argS, Vector3d argT, Vector3d argU, Vector3d argXo) {
      Point3d stu = new Point3d();
      Vector3d temX = new Vector3d(argX);
      temX.sub (argXo); // X - Xo
      // s
      Vector3d temVec = new Vector3d(argT);
      temVec.cross (argU); // T X U
      stu.x = temVec.dot (temX); //T X U * (X -Xo)
      stu.x = stu.x / temVec.dot (argS); // T X U * (X -Xo) / [T X U * S]
      // t
      temVec = new Vector3d(argS);
      temVec.cross (argU); // S X U
      stu.y = temVec.dot (temX); //S X U * (X -Xo)
      stu.y = stu.y / temVec.dot (argT); // S X U * (X -Xo) / [S X U * T]
      // u
      temVec = new Vector3d(argS);
      temVec.cross (argT); // S X T
      stu.z = temVec.dot (temX); //S X T * (X -Xo)
      stu.z = stu.z / temVec.dot (argU); // S X T * (X -Xo) / [S X T * U]

      //PrintData.printVector (stu);
      return stu;
   }

   /**
    * compute FFD for a point
    * @param argX location in natural space
    * @return position of argX in natural space after free-form deformation
    */
   public Point3d computeFFD(Vector3d argX) {
      Vector3d stu = car2stu(argX);
      double [] stuArr = new double [3];
      stu.get (stuArr);
      //System.out.printf ("\ngetXFFD : stu: %f, %f, %f\n", stu.x, stu.y, stu.z);
      Point3d xFFD = new Point3d (myBPoly.getTenProBezCur (stuArr));
      return xFFD;
   }

   /**
    * make FFD for pnt
    * @param pnt
    */
   public Point makeFFD(Point pnt) {
      Point3d pnt3d = pnt.getPosition ();
      Point newPnt = null;
      try {
         newPnt = (Point)pnt.clone ();
      }
      catch (CloneNotSupportedException e) {
         e.printStackTrace();
      }
      newPnt.setPosition (computeFFD(pnt3d));
      return newPnt;
   }

   public void updatePoint(Point pnt) {
      updateP();
      pnt.setPosition (makeFFD(pnt).getPosition ());
      updateRender(pnt);
   }

   public Vertex3d makeFFD(Vertex3d vtx) {
      Vertex3d newV = vtx.copy ();
      newV.setPosition (computeFFD(newV.getPosition ()));
      return newV;
   }

   /**
    * make FFD for vtx
    * @param vtx
    */
   private void updateVertex(Vertex3d vtx) {
      Point3d pnt3d = new Point3d (vtx.getWorldPoint ());
      Point3d newPnt = computeFFD(pnt3d);
      if (!vtx.getMesh ().meshToWorldIsIdentity ()) {
         newPnt.inverseTransform (vtx.getMesh ().getMeshToWorld ());
      }
      vtx.setPosition (newPnt);
   }

   public MeshBase makeFFD(MeshBase mesh) {
      MeshBase newMesh = mesh.copy ();
      updateP();
      for (Vertex3d vtx : newMesh.getVertices ()) {
         updateVertex(vtx);
      }
      return newMesh;
   }

   public void updateMesh (MeshBase mesh) {
      updateP ();
      for (Vertex3d vtx : mesh.getVertices ()) {
         updateVertex(vtx);
      }
   }

   public void updateMeshes () {
      updateP();
      for (MeshBase mesh : myMeshes) {
         HashMap<Point3d, FFDPoint3d> map = myMeshFFDPnts.get (mesh);
         for (Vertex3d vtx : mesh.getVertices ()) {
            FFDPoint3d ffdpnt = map.get (vtx.pnt);
            Point3d newPnt = ffdpnt.computeFFD ();
            if (mesh.meshToWorldIsIdentity ()) {
               vtx.setPosition (newPnt);
            }
            else {
               newPnt.inverseTransform (mesh.getMeshToWorld ());
               vtx.setPosition (newPnt);
            }
         }
      }
      //TODO: may remove later
      if (myMesh != null) {
         updateMesh();
      }
   }



   public void makeIJKDMapping () {
      ijkdMap = new int [NList[0]+1][NList[1]+1][NList[2]+1][3];
      ijkMap = new int [NList[0]+1][NList[1]+1][NList[2]+1];
      int temN = 0;
      int temNN = 0;
      for (int i=0; i < NList[0]+1; i++) {
         for (int j =0; j < NList[1]+1; j++){
            for (int k = 0; k < NList[2] + 1; k++) {
               ijkMap[i][j][k] = temNN;
               temNN++;
               for (int d=0; d < 3; d++) {
                  ijkdMap[i][j][k][d] = temN;
                  temN++;
               }
            }
         }
      }
   }


   public Matrix3d computeJacobian (Vector3d argX) {

      int Ns = NList[0];
      int Nt = NList[1];
      int Nu = NList[2];

      Vector3d Ps = new Vector3d();
      Vector3d dXDs = new Vector3d();
      Vector3d dXDt = new Vector3d();
      Vector3d dXDu = new Vector3d();

      Vector3d X = car2stu(argX);

      double tmpB = 0;
      for (int i=0; i <= Ns-1; i++) {
         for (int j =0; j <= Nt; j++){
            for (int k = 0; k <= Nu; k++) {
               tmpB = BernsteinPolynomia.getBerBasis (Ns-1, i, X.x);
               tmpB *= BernsteinPolynomia.getBerBasis (Nt, j, X.y);
               tmpB *= BernsteinPolynomia.getBerBasis (Nu, k, X.z);
               Ps.sub (controlPnts[i+1][j][k].getPosition (), 
                  controlPnts[i][j][k].getPosition ());
               Ps.scale (tmpB);
               dXDs.add (Ps);
            }
         }
      }
      dXDs.scale (Ns);


      for (int i=0; i <= Ns; i++) {
         for (int j =0; j <= Nt-1; j++){
            for (int k = 0; k <= Nu; k++) {
               tmpB = BernsteinPolynomia.getBerBasis (Ns, i, X.x);
               tmpB *= BernsteinPolynomia.getBerBasis (Nt-1, j, X.y);
               tmpB *= BernsteinPolynomia.getBerBasis (Nu, k, X.z);
               Ps.sub (controlPnts[i][j+1][k].getPosition (), 
                  controlPnts[i][j][k].getPosition ());
               Ps.scale (tmpB);
               dXDt.add (Ps);
            }
         }
      }
      dXDt.scale (Nt);


      for (int i=0; i <= Ns; i++) {
         for (int j =0; j <= Nt; j++){
            for (int k = 0; k <= Nu-1; k++) {
               tmpB = BernsteinPolynomia.getBerBasis (Ns, i, X.x);
               tmpB *= BernsteinPolynomia.getBerBasis (Nt, j, X.y);
               tmpB *= BernsteinPolynomia.getBerBasis (Nu-1, k, X.z);
               Ps.sub (controlPnts[i][j][k+1].getPosition (), 
                  controlPnts[i][j][k].getPosition ());
               Ps.scale (tmpB);
               dXDu.add (Ps);
            }
         }
      }
      dXDt.scale (Nu);

      Matrix3d F = new Matrix3d();
      F.setColumn (0, dXDs);
      F.setColumn (1, dXDt);
      F.setColumn (2, dXDu);
      F.mul (myJacobian);

      return F;
   }

   public Matrix3d computeJacobian1 (Vector3d argX) {

      int Ns = NList[0];
      int Nt = NList[1];
      int Nu = NList[2];

      Vector3d Ps = new Vector3d();
      Vector3d dXDs = new Vector3d();
      Vector3d dXDt = new Vector3d();
      Vector3d dXDu = new Vector3d();

      Vector3d X = car2stu(argX);

      double tmpB = 0;
      for (int i=0; i <= Ns; i++) {
         for (int j =0; j <= Nt; j++){
            for (int k = 0; k <= Nu; k++) {
               tmpB = BernsteinPolynomia.getBerBasisDerivative (Ns, i, X.x);
               tmpB *= BernsteinPolynomia.getBerBasis (Nt, j, X.y);
               tmpB *= BernsteinPolynomia.getBerBasis (Nu, k, X.z);
               Ps.set (controlPnts[i][j][k].getPosition ());
               Ps.scale (tmpB);
               dXDs.add (Ps);
            }
         }
      }


      for (int i=0; i <= Ns; i++) {
         for (int j =0; j <= Nt; j++){
            for (int k = 0; k <= Nu; k++) {
               tmpB = BernsteinPolynomia.getBerBasis (Ns, i, X.x);
               tmpB *= BernsteinPolynomia.getBerBasisDerivative (Nt, j, X.y);
               tmpB *= BernsteinPolynomia.getBerBasis (Nu, k, X.z);
               Ps.set (controlPnts[i][j][k].getPosition ());
               Ps.scale (tmpB);
               dXDt.add (Ps);
            }
         }
      }


      for (int i=0; i <= Ns; i++) {
         for (int j =0; j <= Nt; j++){
            for (int k = 0; k <= Nu; k++) {
               tmpB = BernsteinPolynomia.getBerBasis (Ns, i, X.x);
               tmpB *= BernsteinPolynomia.getBerBasis (Nt, j, X.y);
               tmpB *= BernsteinPolynomia.getBerBasisDerivative (Nu, k, X.z);
               Ps.set (controlPnts[i][j][k].getPosition ());
               Ps.scale (tmpB);
               dXDu.add (Ps);
            }
         }
      }

      Matrix3d F = new Matrix3d();
      F.setColumn (0, dXDs);
      F.setColumn (1, dXDt);
      F.setColumn (2, dXDu);
      F.mul (myJacobian);

      return F;
   }

   /**
    * mapping material space to natural space
    * @return
    */
   public Matrix3d computeX2STUJacobian () {
      Matrix3d Jac = new Matrix3d();
      Vector3d dsDX = new Vector3d();
      Vector3d dtDX = new Vector3d();
      Vector3d duDX = new Vector3d();

      double tmp = 0;
      // s
      Vector3d temVec = new Vector3d(myT);
      temVec.cross (myU); // T X U
      tmp = temVec.dot (myS); // T X U * S
      tmp = 1.0/tmp;
      temVec.scale (tmp); // T X U / [T X U * S]
      dsDX.set (temVec);
      // t
      temVec = new Vector3d(myS);
      temVec.cross (myU); // S X U
      tmp = temVec.dot (myT); //S X U * T
      tmp = 1.0/tmp;
      temVec.scale (tmp); // S X U / [S X U * T]
      dtDX.set (temVec);
      // u
      temVec = new Vector3d(myS);
      temVec.cross (myT); // S X T
      tmp = temVec.dot (myU); //S X T * U
      tmp = 1.0/tmp;
      temVec.scale (tmp); // S X T / [S X T * U]
      duDX.set (temVec);

      Jac.setRow (0, dsDX);
      Jac.setRow (1, dtDX);
      Jac.setRow (2, duDX);

      myJacobian.set (Jac);
      return Jac;
   }


   // -------------------------------------
   // implement rendering
   // -------------------------------------

   protected void makeEdge() {
      ArrayList<int []> ins = new ArrayList<int []>();
      int [] in;
      for (int i = 0; i <= NList[0]; i++) {
         for (int j = 0; j <= NList[1]; j++) {
            for (int k = 0; k <= NList[2]; k++) {
               if (i != NList[0]) {
                  in = new int [2];
                  in [0] = controlPnts[i][j][k].myIndex;
                  in [1] = controlPnts[i+1][j][k].myIndex;
                  ins.add (in);

               }
               if (j != NList[1]) {
                  in = new int [2];
                  in [0] = controlPnts[i][j][k].myIndex;
                  in [1] = controlPnts[i][j+1][k].myIndex;
                  ins.add (in);
               }
               if (k != NList[2]) {
                  in = new int [2];
                  in [0] = controlPnts[i][j][k].myIndex;
                  in [1] = controlPnts[i][j][k+1].myIndex;
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

   private void setControlPointsRenderProps() {
      controlPointsRenderProps = controlPntList.getRenderProps ();
      controlPointsRenderProps.setPointColor (new Color (0.2f, 0.6f, 1.0f));
      controlPointsRenderProps.setPointStyle (PointStyle.SPHERE);
      controlPointsRenderProps.setPointRadius (myS.norm ()*0.02);
      RenderProps.setLineColor (this, new Color (0.2f, 0.6f, 1.0f));
   }

   public RenderProps createRenderProps() {
      return RenderProps.createLineProps (this);
   }

   public void prerender (RenderList list) {
      super.prerender (list);
      list.addIfVisible (controlPntList);
      controlPntList.prerender (list);
   }

   @Override
   public void render (Renderer renderer, int flags) {
      renderEdges (renderer);
   }

   public void renderEdges (Renderer renderer) {
      for (int i=0; i<edgeVtxIdx.length; i++) {
         renderer.drawLine (myRenderProps, 
            controlPntList.get (edgeVtxIdx[i][0]).myRenderCoords, 
            controlPntList.get (edgeVtxIdx[i][1]).myRenderCoords, 
            isSelected ());
      }
   }

   @Override
   public StepAdjustment advance (double t0, double t1, int flags) {
      updateMeshes();
      return null;
   }

   public void updateRender() {
      updateRender(this);
   }

   public static void updateRender(ModelComponent mc) {
      RootModel root = RootModel.getRoot (mc);
      if (root != null) {
         root.rerender();
      }
   }


   // -------------------------------------
   // implement action listener
   // -------------------------------------


   @Override
   public void actionPerformed(ActionEvent event) {
      if (event.getActionCommand().equals ("apply FFD")) {
         applyFFD();
      }
      else if (event.getActionCommand().equals ("undo FFD")) {
         undoFFD();
      }
      else if (event.getActionCommand().equals ("reset FFD grid")) {
         resetFFDGrid();
      }
      else if (event.getActionCommand ().equals ("advance meshes")) {
         advanceMeshes();
      }
   }

   private void applyFFD() {
      resetTime();
      updateMeshes();
   }

   private void undoFFD() {
      resetTime();
      resetFFD();
      updateMeshes();
   }

   private void resetFFDGrid() {
      resetTime();
      resetFFD();
      updateRender();
   }

   private void advanceMeshes() {
      resetTime();
      advanceMeshInitialStates();
   }

   private void resetTime() {
      RootModel root = RootModel.getRoot (this);
      if (root != null) {
         root.getWayPoint(0).setValid (false);
         Main main = Main.getMain();
         if (main != null) {
            main.reset();
         }
      }
   }

   public boolean getMenuItems(List<Object> items) {
      JMenuItem menuItem;
      menuItem = makeMenuItem ("apply FFD", "deform the model");
      items.add (menuItem);
      items.add (makeMenuItem ("reset FFD grid", "reset the deformation grid"));
      menuItem = makeMenuItem ("undo FFD", "unfo the deformation");
      items.add (menuItem);
      menuItem = makeMenuItem ("advance mesh", "advance mesh material space to current natural state");
      items.add (menuItem);
      return true;
   }   

   public JMenuItem makeMenuItem (String cmd, String toolTip) {
      JMenuItem item = new JMenuItem(cmd);
      item.addActionListener(this);
      item.setActionCommand(cmd);
      if (toolTip != null && !toolTip.equals ("")) {
         item.setToolTipText (toolTip);
      }
      return item;
   }

   public boolean hierarchyContainsReferences() {
      return false;
   }

   // -------------------------------------
   // implement action listener
   // -------------------------------------

   public ControlPanel createControlPanel() {
      ControlPanel panel = new ControlPanel();
      panel.addLabel ("Bezier-Bernstein FFD");
      panel.addWidget (
         "grid nodes visible", controlPntList, "renderProps.visible");
      panel.addWidget (
         "grid visible", this, "renderProps.visible");
      return panel;
   }


   //------------------------------------------------old method---------------------------------------//
   protected MeshBase myMesh; // input
   private ArrayList<FFDPoint3d> materialPointList = new ArrayList<FFDPoint3d> ();

   private MeshConvexHull myConvexHull;



   // automatic grid generate method
   public BezierFFD3d (int [] argNList, MeshBase mesh) {
      setNList(argNList);
      setMesh(mesh);
      setName(DEFAULT_NAME);
      regenerateFFDGrid();
   }

   // automatic grid generate method
   public BezierFFD3d (int argNX, int argNY, int argNZ, MeshBase mesh) {
      setNList(argNX, argNY, argNZ);
      setMesh(mesh);
      setName(DEFAULT_NAME);
      regenerateFFDGrid();
   }


   /**
    * set mesh without updating mesh 
    * FFD initial state
    * @param mesh
    */
   public void setMesh(MeshBase mesh) {
      myMesh = mesh;
   }

   /**
    * set mesh and update mesh 
    * FFD initial state
    * @param mesh
    */
   public void reDefineMesh(MeshBase mesh) {
      myMesh = mesh;
      advanceMeshInitialState();
   }

   public MeshBase getMesh() {
      return myMesh;
   }

   /**
    * set current mesh as initial state
    * update mesh material space
    * save 
    */
   public void advanceMeshInitialState() {
      if (myMesh != null) {
         setAsUndeformedMesh(myMesh);
      }
   }


   private void setAsUndeformedMesh(MeshBase mesh) {
      clearUndeformedMesh();
      for (Vertex3d vtx : mesh.getVertices ()) {
         addToUndeformedMesh(vtx);
      }
   }

   private void addToUndeformedMesh (Vertex3d vtx) {
      FFDPoint3d pnt3d = new FFDPoint3d (car2stu (vtx.getPosition ()));
      materialPointList.add (pnt3d);
   }

   private void clearUndeformedMesh() {
      materialPointList.clear ();
   }

   /**
    * generate convex hull for embedded mesh
    */
   public void generateConvexHull () {
      myConvexHull = new MeshConvexHull (myMesh);
   }


   /**
    * compute distance between a point and it's projected position 
    * on convex hull of embedded mesh.
    * @param pnt input mesh
    * @return distance; 
    * if <tt>pnt</tt> is inside convex hull of the embedded mesh, the
    * distance would be negative otherwise it's positive;
    */
   public double computeDistanceToConvexHull (Point3d pnt) {
      if (myConvexHull == null) {
         throw new ImproperStateException ("Convex hull not initialized!");
      }
      double dis = myConvexHull.computeDistanceToConvexHull (pnt);
      return dis;
   }

   /**
    * compute distance between the given point and its nearest point 
    * on the mesh embedded in FFD.
    * @param pnt
    * @return distance
    */
   public double computeDistanceToMesh (Point3d pnt) {
      BVFeatureQuery bv = new BVFeatureQuery ();
      Point3d nearPnt = new Point3d ();
      if (myMesh instanceof PolygonalMesh) {
         PolygonalMesh mesh = (PolygonalMesh) myMesh;
         if (mesh .isTriangular ()) {
            bv.nearestFaceToPoint (nearPnt, null, mesh, pnt);
         }
         else {
            mesh = (PolygonalMesh) myMesh.clone ();
            mesh.triangulate ();
            bv.nearestFaceToPoint (nearPnt, null, mesh, pnt);
         }
      }
      else {
         BVTree tree = new OBBTree (myMesh);
         nearPnt.set (bv.nearestVertexToPoint (tree, pnt).getPosition ());
      }
      return nearPnt.distance (pnt);
   }

   /**
    * 
    * @return convex hull of undeformed embedded mesh
    */
   public MeshConvexHull getConvexHull () {
      return myConvexHull;
   }

   public void updateMesh() {
      for (int i = 0; i < myMesh.getVertices ().size (); i++) {
         double [] tmp = new double [3];
         materialPointList.get (i).get (tmp);
         Point3d xFFD = new Point3d (myBPoly.getTenProBezCur (tmp));
         myMesh.getVertex (i).setPosition (xFFD);
      }
   }





}
