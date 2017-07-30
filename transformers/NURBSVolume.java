package artisynth.models.swallowingRegistrationTool.transformers;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.Reader;
import java.util.ArrayList;

import maspack.geometry.NURBSCurve3d;
import maspack.geometry.NURBSObject;
import maspack.geometry.io.WavefrontReader;
import maspack.matrix.ImproperSizeException;
import maspack.matrix.LUDecomposition;
import maspack.matrix.Matrix3d;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.Vector4d;
import maspack.matrix.VectorNd;
import maspack.render.*;
import maspack.render.Renderer.DrawMode;
import maspack.util.NumberFormat;

import javax.media.opengl.GL2;
import javax.media.opengl.GLDrawable;

import artisynth.models.frank2.frankUtilities.StopWatch;

/**
 * 
 * @author KeyiTang
 *
 */
public class NURBSVolume extends NURBSObject {

   protected int [] numCtrlPntsUVT = new int [3];
   protected int [] degreesUVT = new int [3];
   
   protected double [][] restCtrlPnts;
   

   public NURBSVolume () {
      super();
   }

   /**
    * Creates a NURBS surface using degree and knot values in the u and v
    * directions, along with control points. The surface may be open or closed
    * in either the u or v directions. For information on how these arguments
    * are constrained, see the general version of
    * {@link #set(int,int,double[],int,int,double[],Vector4d[]) set}.
    * 
    * @param du
    * degree in the u direction
    * @param typeu
    * u direction type, which must be either {@link NURBSCurve3d#OPEN OPEN} or
    * {@link NURBSCurve3d#CLOSED CLOSED}.
    * @param knotsu
    * knot values for the u direction
    * @param dv
    * degree in the v direction
    * @param typev
    * v direction type, which must be either {@link NURBSCurve3d#OPEN OPEN} or
    * {@link NURBSCurve3d#CLOSED CLOSED}.
    * @param knotsv
    * knot values for the v direction
    * @param ctrlPnts
    * control points
    * @throws IllegalArgumentException
    * if constraints on the arguments are violated
    * @see #set(int,int,double[],int,int,double[],Vector4d[])
    */
   public NURBSVolume (int [] degrees, double [][] knots, Vector4d [] ctrlPnts) {
      this ();
      set (degrees, knots, ctrlPnts);
   }

   /**
    * Sets this NURBS surface using degree and knot values in the u and v
    * directions, along with control points. The surface may be open or closed
    * in either the u or v directions.
    * 
    * <p>
    * Let du, numku, and numcu be the degree, number of knots, and number of
    * control points associated with the u direction. The degree must be 1 or
    * greater. If the u direction is open, then <blockquote> numcu = numku - du +
    * 1 </blockquote> and if it is closed, <blockquote> numcu = numku - 2*du + 1
    * </blockquote> Analogous results hold for the v direction. The total number
    * of control points is numcu*numcv, and so the ctrlPnts argument must be at
    * least this length. Control points should be arranged so that the first set
    * of v control points comes first, followed by the second set, etc.
    * 
    * <p>
    * This method automatically sets ustart, uend, vstart, and vend (see
    * {@link #setRangeU setRangeU} and {@link #setRangeV setRangeV}) to to
    * knotsu[du-1], knotsu[numku-du], knotsv[dv-1], and knotsv[numkv-dv].
    * 
    * <p>
    * The control points are specified as 4-vectors, where their spatial
    * location is given by x, y, and z and their weight is given by w. The
    * points should not be in homogeneous form; i.e., x, y, and z should not be
    * premultipled by w.
    * 
    * @param du
    * degree in the u direction
    * @param typeu
    * u direction type, which must be either {@link NURBSCurve3d#OPEN OPEN} or
    * {@link NURBSCurve3d#CLOSED CLOSED}.
    * @param knotsu
    * knot values for the u direction
    * @param dv
    * degree in the v direction
    * @param typev
    * v direction type, which must be either {@link NURBSCurve3d#OPEN OPEN} or
    * {@link NURBSCurve3d#CLOSED CLOSED}.
    * @param knotsv
    * knot values for the v direction
    * @param ctrlPnts
    * control points
    * @throws IllegalArgumentException
    * if constraints on the arguments are violated
    */
   public void set (int [] degreesUVT, double [][] knotsUVT, Vector4d [] ctrlPnts) {
      if (degreesUVT.length !=3) {
         throw new IllegalArgumentException ("The dimension of degrees has to be 3");
      }
      if (knotsUVT.length != 3) {
         throw new IllegalArgumentException ("The dimension of knots has to be 3");
      }

      String [] dirs = {"u: ", "v: ", "s: "};
      for (int i = 0; i < 3; i++) {
         checkKnotConsistency (dirs[i], degreesUVT[i], knotsUVT[0]);
      }

      int numCtrlPnts = 1;
      for (int i = 0; i < 3; i ++) {
         // m = n + p + 1
         numCtrlPntsUVT[i] = (knotsUVT[i].length - degreesUVT[i] - 1);
         numCtrlPnts *= numCtrlPntsUVT[i];
      }
      if (ctrlPnts.length != numCtrlPnts) {
         throw new IllegalArgumentException ("incompatible control points size");
      }

      setKnots (knotsUVT, degreesUVT);
      setControlPoints (ctrlPnts, numCtrlPnts);
      updateRestState ();
   }

   private void checkKnotConsistency (String msgPrefix, int degree, double[] knots) {
      if (degree < 1) {
         throw new IllegalArgumentException (
            msgPrefix + "degree is less than one");
      }
      for (int i = 0; i < knots.length - 1; i++) {
         if (knots[i] > knots[i + 1]) {
            throw new IllegalArgumentException (
               msgPrefix + "knots are not monotonically increasing");
         }
      }
   }


   public void setDegrees (int [] degrees) {
      if (degrees.length != 3) {
         throw new IllegalArgumentException (
         "Dimension must be 3!");
      }
      for (int i = 0; i < 3; i++) {
         degreesUVT[i] = degrees[i];
      }
   }

   public void setCtrlPntsNum (int [] numCtrlPnts) {
      if (numCtrlPnts.length != 3) {
         throw new IllegalArgumentException (
         "Dimension must be 3!");
      }
      for (int i = 0; i < 3; i++) {
         numCtrlPntsUVT [i] = numCtrlPnts[i];
      }
   }
   
   public void setCtrlPntsNum (int numU, int numV, int numT) {
      numCtrlPntsUVT[0] = numU;
      numCtrlPntsUVT[1] = numV;
      numCtrlPntsUVT[2] = numT;
   }
   
   public int [] getCtrlPntsNum () {
      return numCtrlPntsUVT.clone ();
   }
   
   public void setDegrees (int degreeU, int degreeV, int degreeT) {
      degreesUVT[0] = degreeU;
      degreesUVT[1] = degreeV;
      degreesUVT[2] = degreeT;
   }
   
   public int [] getDegrees () {
      return degreesUVT.clone ();
   }
   
   /**
    * This method will advance rest position of control points 
    * to their current position. 
    * <p>
    * The change of rest position will change the embedding 
    */
   protected void updateRestState () {
      restCtrlPnts = new double [myCtrlPnts.size ()][4];
      for (int i = 0; i < restCtrlPnts.length; i++) {
         for (int j = 0; j < 4; j++) {
            restCtrlPnts[i][j] = myCtrlPnts.get (i).get (j);
         }
      }
   }
   
   public Point3d getCtrlPointRestPosition (int idx) {
      Point3d position = new Point3d ();
      position.x = restCtrlPnts[idx][0];
      position.y = restCtrlPnts[idx][1];
      position.z = restCtrlPnts[idx][2];
      return position;
   }
   
   public Vector4d getCtrlPointRestState (int idx) {
      Vector4d rest = new Vector4d ();
      rest.x = restCtrlPnts[idx][0];
      rest.y = restCtrlPnts[idx][1];
      rest.z = restCtrlPnts[idx][2];
      rest.w = restCtrlPnts[idx][3];
      return rest;
   }


   /**
    * Evaluates the point on this surface corresponding to u and v. These values
    * are clipped, if necessary, to the range [ustart, uend] and [vstart, vend].
    * 
    * @param pnt
    * returns the surface point value
    * @param u
    * u parameter value
    * @param v
    * v parameter value
    */
   public void eval (Point3d pnt, Point3d uvt) {
      // initialize basis
      double [][] basis = new double [3][];
      for (int i = 0; i < 3; i++) {
         basis[i] = new double [degreesUVT[i] + 1];
      }

      // initialize knots
      int [] t0 =  new int [3];
      // initialize degrees
      int [] degrees = new int [3];
      //System.out.println ("");
      //System.out.println ("uvt: " + uvt);
      for (int i = 0; i < 3; i++) {
         t0[i] = findSpan (uvt.get (i), KnotsUVT[i], degreesUVT[i]);
         evalBasis (basis[i], uvt.get (i), t0[i], KnotsUVT[i], degreesUVT[i]);
         degrees [i] = degreesUVT[i];
      }

      pnt.setZero();
      //System.out.println ("span: " + new Vector3d(t0[0], t0[1], t0[2]));
      double w = 0;
      for (int i = 0; i <= degrees[0]; i++) {
         int d_i = getCtrlIndex (t0[0], i, degreesUVT[0]);
         double B_i = basis[0][i];
         for (int j = 0; j <= degrees[1]; j++) {
            int d_j = getCtrlIndex (t0[1], j, degreesUVT[1]);
            double B_j = basis[1][j];
            for (int k = 0; k <= degrees[2]; k++) {
               int d_k = getCtrlIndex (t0[2], k, degreesUVT[2]);
               double B_k = basis[2][k];
               //System.out.println ("");
               //System.out.printf ("index: %d %d %d ", d_i, d_j, d_k);
               //System.out.println (indexMap(d_i, d_j, d_k));
               Vector4d cpnt = myCtrlPnts.get(indexMap(d_i, d_j, d_k));
               double wbb = cpnt.w * B_i * B_j * B_k;
               pnt.x += wbb * cpnt.x;
               pnt.y += wbb * cpnt.y;
               pnt.z += wbb * cpnt.z;
               w += wbb;
            }
         }
      }
      pnt.scale (1 / w);
      //System.out.println (w);
   }


   public int indexMap (int uIdx, int vIdx, int tIdx) {
      int idx = uIdx * numCtrlPntsUVT[2]*numCtrlPntsUVT[1] + vIdx * numCtrlPntsUVT[2] + tIdx;
      return idx;
   }

   public int [] inverseIndexMap (int index) {
      int numU = numCtrlPntsUVT[0];
      int numV = numCtrlPntsUVT[1];
      int numT = numCtrlPntsUVT[2];
      
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

   /**
    * {@inheritDoc}
    */
   public void render (Renderer renderer, RenderProps props, int flags) {
      boolean selecting = renderer.isSelecting();
      
      if (numControlPoints() == 0) {
         return;
      }

      // draw the control points
      //if (myDrawControlShapeP) {
         //drawControlPoints (renderer, props, flags);
      //}         

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
               Vector4d cpnt = myCtrlPnts.get(indexMap(i, j, k));
               renderer.addVertex (cpnt.x, cpnt.y, cpnt.z);
            }
            renderer.endDraw ();
         }
      }

      for (int i = 0; i < numCtrlPntsUVT[0]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[2]; j++) { // pnt.setFromHomogeneous


            renderer.beginDraw (DrawMode.LINE_STRIP);

            for (int k = 0; k < numCtrlPntsUVT[1]; k++) {
               Vector4d cpnt = myCtrlPnts.get(indexMap(i, k, j));
               renderer.addVertex (cpnt.x, cpnt.y, cpnt.z);
            }
            renderer.endDraw ();
         }
      }

      for (int i = 0; i < numCtrlPntsUVT[1]; i++) {
         for (int j = 0; j < numCtrlPntsUVT[2]; j++) { // pnt.setFromHomogeneous

            renderer.beginDraw (DrawMode.LINE_STRIP);

            for (int k = 0; k < numCtrlPntsUVT[0]; k++) {
               Vector4d cpnt = myCtrlPnts.get(indexMap (k, i, j));
               renderer.addVertex (cpnt.x, cpnt.y, cpnt.z);
            }
            renderer.endDraw ();
         }
      }

      renderer.setHighlighting (false);
   }


   /**
    * {@inheritDoc}
    */
   public RenderProps createRenderProps() {
      return new MeshRenderProps();
   }

   private void setKnots (double knots[][], int [] degrees) {
      for (int i = 0; i < 3; i ++) {
         degreesUVT[i] = degrees[i];
         KnotsUVT[i] = new double [knots[i].length];
         for (int j = 0; j < knots[i].length; j++) {
            KnotsUVT[i][j] = knots[i][j];
         }
      }
   }

   @Override
   public void write (PrintWriter pw, String fmtStr, boolean relative)
   throws IOException {
      // TODO Auto-generated method stub

   }

   public static int getCtrlIndex (int k, int i, int degree) {
      return (k - degree + i);
   }

   public static double [] evalBasis (double [] vals, double u, 
      int span, double [] knots, int degree) {
      if (vals.length < degree+1) {
         throw new ImproperSizeException ("Nonvanishing basis size is (degree+1)");
      }
      vals[0] = 1.0;

      for (int j = 1; j <= degree; j++) {
         double val = 0;
         for (int r = 0; r < j; r++) {
            double tmp = vals[r]/(right(u, knots, span, r+1) + left(u, knots, span, j-r));
            vals[r] = val + right(u, knots, span, r+1) * tmp;
            val = left (u, knots, span, j-r) * tmp;
         }
         vals[j] = val;
      }
      //System.out.println (new VectorNd (vals));
      return vals;
   }

   protected static double [][] allBasis (double u, int span, int degreeUpper, double [] knots) {

      // N_j_r, 
      // j is degree[0, degreeUpper]
      // r [0, j]
      double [][] vals = new double [degreeUpper+1][];
      vals[0] = new double [1];
      vals[0][0] = 1.0;


      for (int j = 1; j <= degreeUpper; j++) {
         double val = 0;
         vals[j] = new double [j+1];
         for (int r = 0; r < j; r++) {
            double tmp = vals[j-1][r]/(right(u, knots, span, r+1) + left(u, knots, span, j-r));
            vals[j][r] = val + right(u, knots, span, r+1) * tmp;
            val = left (u, knots, span, j-r) * tmp;
         }
         vals[j][j] = val;
      }

      return vals;
   }

   private static double left (double u, double [] knots, int i, int j) {
      return (u - knots[i+1-j]);
   }

   private static double right (double u, double [] knots, int i, int j) {
      return (knots[i+j] - u);
   }

   protected double [][] KnotsUVT = new double [3][];


   public static int findSpan (double u, double [] knots, int degree) {
      /* Determine the knot span index */
      /* Return: the knot span index */
      int i = degree;
      if (knots[i] == u) {
         return i;
      }
      while (i < (knots.length-degree-1)) {
         if (knots[i] <= u && u < knots[i+1] && knots[i] < knots[i+1]) {
            return i;
         }
         i++;
      } 
      return (knots.length - degree - 2);
   }

   /**
    * Create clamped uniform knots vector, thus length of the knots vector is 
    * <blockquote>
    * <tt>numCtrlPnts</tt> + <tt>degree</tt> + 1;
    * </blockquote>
    * The knots vector form:
    * <p>
    * [0 to <tt>degree</tt>]: 0 ;<p>
    * [<tt>degree+1</tt> to <tt>numCtrlPnts-1</tt>]: <tt>i / (numCtrlPnts-degree</tt>);<p>
    * [<tt>numCtrlPnts</tt> to <tt>end</tt>] : 1;<p>
    * 
    * 
    * @param degree
    * @param numCtrlPnts
    * @return
    */
   public static double [] createUniformKnotsVector (int degree, int numCtrlPnts) {
      if (degree < 1 || degree > numCtrlPnts) {
         throw new IllegalArgumentException ("Incompatible degree and control point number!");
      }
      double[] knots = new double[numCtrlPnts+degree+1];
      //double unit = 1.0 / ((double)(numCtrlPnts - degree));
      double unit = 1.0;
      double knot = 0;
      for (int i=0; i<knots.length; i++) {
         if (i <= degree) {
            knots[i] = 0;
         }
         else if (i >= numCtrlPnts) {
            knots[i] = numCtrlPnts -degree;
         }
         else {
            knot += unit;
            knots[i] = knot;
         }
      }
      return knots;
   }


   // just for testing
   public static void main (String [] args) {
      double [] knots = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5};
      int span = 7;
      double u = 4.6;
      double [] vals = new double [3];
      NURBSVolume.evalBasis (vals, u, span, knots, 2);
      NURBSVolume.allBasis (u, span, 2, knots);
      System.out.println (new VectorNd (vals));
      //System.out.println (NURBSVolume.findSpan (u, knots, 2));
   }

   protected final double U_SEARCH_TOL = 1e-8;

   protected double golden3d (Point3d uvt, Point3d pnt, Point3d minUVT, Point3d maxUVT) {
      double r = (Math.sqrt (5) - 1) / 2.0;
      Point3d [] ABCD = new Point3d [4];

      ABCD[0] = new Point3d (minUVT);
      ABCD[1] = new Point3d (maxUVT);

      Point3d tmp = new Point3d ();
      tmp.sub (ABCD[1], ABCD[0]);
      tmp.scale (r);
      ABCD[2] = new Point3d (maxUVT);
      ABCD[2].sub (tmp);
      ABCD[3] = new Point3d (minUVT);
      ABCD[3].add (tmp);

      Point3d result = new Point3d();
      double err = Double.MAX_VALUE;
      while (ABCD[0].distance (ABCD[1]) > U_SEARCH_TOL) {
         Point3d fcpnt = new Point3d ();
         Point3d fdpnt = new Point3d ();
         eval (fcpnt, ABCD[2]);
         eval (fdpnt, ABCD[3]);
         fcpnt.sub (pnt);
         fdpnt.sub (pnt);
         fcpnt.absolute ();
         fdpnt.absolute ();

         for (int i = 0; i < 3; i++) {
            double [] abcd = new double [4];
            for (int j = 0; j < 4; j++) {
               abcd[j] = ABCD[j].get (i);
            }
            sigRun (abcd, fcpnt.get (i), fdpnt.get (i));
            for (int j = 0; j < 4; j++) {
               ABCD[j].set (i, abcd[j]);
            }
         }
         result.setZero ();
         result.add (ABCD[0], ABCD[1]);
         result.scale (0.5);
         Point3d p = new Point3d();
         eval (p, result);
         err = p.distance (pnt);
         System.out.print (" "+ err);
      }
      uvt.set (result);
      return err;
   }


   private void sigRun (double [] abcd, double fc, double fd) {
      double r = (Math.sqrt (5) - 1) / 2.0;

      double a = abcd[0];
      double b = abcd[1];
      double c = abcd[2];
      double d = abcd[3];

      if (fc < fd) {
         b = abcd[3];
      }
      else {
         a = abcd[2];
      }

      c = b - r * (b - a);
      d = a + r * (b - a);

      abcd[0] = a;
      abcd[1] = b;
      abcd[2] = c;
      abcd[3] = d;
   }

   static final double DEFAULT_STEP_SIZE = 1;
  
   /**
    * Gradient descent  method
    * slow and converge to local minimum
    * @param uvt
    * @param pntInWorld
    * @param startUVT Initial guess, if null, start from zero point;
    */
   public double findUVTGD (Point3d uvt, Point3d pntInWorld, Point3d startUVT) {
      //TODO: 

      Point3d u0 = new Point3d();
      if (startUVT != null) {
         u0.set (startUVT);
      }
      
      System.out.println (u0);

      double r = DEFAULT_STEP_SIZE;

      Point3d [][][] pnts = new Point3d [numCtrlPntsUVT[0]][numCtrlPntsUVT[1]][numCtrlPntsUVT[2]];
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

      Point3d step = new Point3d (Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      Point3d guess = new Point3d ();
      Point3d residual = new Point3d ();
      eval (guess, u0);
      residual.sub (guess, pntInWorld);
      double oldErr = Double.MAX_VALUE;
      Point3d pru = new Point3d(u0);
      double err = 0;

      Point3d [][][] Jac;

      for  (int iteration = 0; iteration < 1E3; iteration++) {

         Jac = evalDeri (u0, degreesUVT, KnotsUVT, 1, pnts);

         step.x = Jac[1][0][0].dot (residual)/2.0;
         step.y = Jac[0][1][0].dot (residual)/2.0;
         step.z = Jac[0][0][1].dot (residual)/2.0;
         step.scale (-r);
         pru.set (u0);
         u0.add (step);
         eval (guess, u0);
         residual.sub (guess, pntInWorld);
         err = residual.norm ();

         if (err > oldErr) {
            err = adjustStepSize (pru, step, pntInWorld, oldErr);
            if (err == -1) {
               System.err.println ("Failed to find UVT -1!");
               return -1;
            }
            u0.set (pru);
            eval (guess, u0);
            residual.sub (guess, pntInWorld);
         }

         oldErr = err;
         if (err < U_SEARCH_TOL) {
            uvt.set (u0);
            System.out.println (err);
            System.out.println(uvt);
            return err;
         }

         for (int i = 0; i < 3; i++) {
            if (u0.get (i) < 0 || u0.get (i) > KnotsUVT[i][KnotsUVT[i].length-1]) {
               System.err.println ("Out of bound!");
               return -1;
            }
         }
       //System.out.println (iteration);
      }

      System.err.println (err);
      uvt.set (u0);
      System.out.println(uvt);

      //System.out.println (err);
      return err;
   }
   
   /**
    * Newton method
    * @param uvt
    * @param pntInWorld
    * @param startUVT Initial guess, if null, start from zero point;
    */
   public double findUVT (Point3d uvt, Point3d pntInWorld, Point3d startUVT) {
      //TODO: 

      Point3d u0 = new Point3d();
      if (startUVT != null) {
         u0.set (startUVT);
      }
      

      double r = DEFAULT_STEP_SIZE;

      Point3d [][][] pnts = new Point3d [numCtrlPntsUVT[0]][numCtrlPntsUVT[1]][numCtrlPntsUVT[2]];
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

      Point3d step = new Point3d (Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      Point3d guess = new Point3d ();
      Point3d residual = new Point3d ();
      double oldErr = Double.MAX_VALUE;
      Point3d pru = new Point3d(u0);
      double err = 0;

      Point3d [][][] jac;

      for  (int iteration = 0; iteration < 1E3; iteration++) {

         jac = evalDeri (u0, degreesUVT, KnotsUVT, 1, pnts);
         step.x = jac[1][0][0].dot (residual)/2.0;
         step.y = jac[0][1][0].dot (residual)/2.0;
         step.z = jac[0][0][1].dot (residual)/2.0;
         
         Matrix3d Jac = new Matrix3d ();
         Jac.setColumn (0, jac[1][0][0]);
         Jac.setColumn (1, jac[0][1][0]);
         Jac.setColumn (2, jac[0][0][1]);
         
         LUDecomposition LUD = new LUDecomposition();
         LUD.factor (Jac);
         double cond = LUD.conditionEstimate (Jac);
         if (cond > 1e10)
            System.err.println (
               "Warning: condition number for solving natural coordinates is "
               + cond);
         // solve Jacobian to obtain an update for the coords
         LUD.solve (step, residual);
         pru.set (u0);
         u0.sub (step);
         eval (guess, u0);
         residual.sub (guess, pntInWorld);
         err = residual.norm ();

         if (err > oldErr) {
            err = adjustStepSize (pru, step, pntInWorld, oldErr);
            if (err == -1) {
               System.err.println ("Failed to find UVT -1!");
               return -1;
            }
            u0.set (pru);
            eval (guess, u0);
            residual.sub (guess, pntInWorld);
         }

         oldErr = err;
         if (err < U_SEARCH_TOL) {
            uvt.set (u0);
            //System.out.println (err);
            //System.out.println(uvt);
            return err;
         }

         /*
         for (int i = 0; i < 3; i++) {
            if (u0.get (i) < 0 || u0.get (i) > KnotsUVT[i][KnotsUVT[i].length-1]) {
               System.err.println ("Out of bound!");
               return -1;
            }
         }*/
       //System.out.println (iteration);
      }
      uvt.set (u0);

      System.err.println (err);
      return err;
   }

   private double adjustStepSize (Point3d uvt, Point3d step, Point3d pnt, double prErr) {
      double err = 0;
      double eps = 1e-12;
      // start by limiting del to a magnitude of 1
      if (step.norm() > 1) {
         step.normalize();  
      }
      // and keep cutting the step size in half until the residual starts
      // dropping again
      double alpha = 0.5;
      Point3d uvt1 = new Point3d ();
      while (alpha > eps && err > prErr) {
         Point3d guess = new Point3d ();
         uvt1.scaledAdd (-alpha, step, uvt);
         eval (guess, uvt1);
         err = guess.distance (pnt);
         alpha *= 0.5;
      }
      uvt.set (uvt1);
      if (alpha < eps) {
         return -1;  // failed
      }
      return err;
   }
  

   protected Matrix3d evalJacUVT (Point3d uvt, Point3d [][][] CtrlPnts) {
      Point3d [][][] deri;
      deri = evalDeri (uvt, degreesUVT, KnotsUVT, 1, CtrlPnts);

      Matrix3d Jac = new Matrix3d ();
      Jac.setColumn (0, deri[1][0][0]);
      Jac.setColumn (1, deri[0][1][0]);
      Jac.setColumn (2, deri[0][0][1]);

      return Jac;
   }


   /**
    * Given a point in parametric coordinates, evaluate Jacobian matrix 
    * @param uvt point in parametric coordinates
    * @return Jac
    */
   public Matrix3d evalJacUVT (Point3d uvt) {

      Point3d [][][] pnts = new Point3d [numCtrlPntsUVT[0]][numCtrlPntsUVT[1]][numCtrlPntsUVT[2]];
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
      return evalJacUVT(uvt, pnts);
   }

   // TODO: just work for non-rational B-spline for now
   // need incorporate derivative method for rational B-spline
   public static Point3d[][][] evalDeri (Point3d uvt, int [] degrees, double [][] knots, 
      int deriDepth, Point3d [][][] CtrlPnts) {

      int [] du = new int [3];
      for (int i = 0; i < 3; i++) {
         du[i] = Math.min (degrees[i], deriDepth);
      }

      // if derivative depth is larger than curve degree, 
      // zero values were set as null pointer for implicit
      // use.
      Point3d [][][] VKD = new Point3d [deriDepth+1][deriDepth+1][deriDepth+1];
      int [] spans = new int [3];

      // direction, derivative order, non-vanishing value indices
      double [][][] basis = new double [3][][];
      for (int i = 0; i < 3; i++) {
         spans[i] = findSpan (uvt.get (i), knots[i], degrees[i]);
         basis[i] = allBasis (uvt.get (i), spans[i], degrees[i], knots[i]);
      }

      int [] r0 = {spans[0]-degrees[0], spans[1]-degrees[1], spans[2]-degrees[2]};
      Point3d [][][][][][] PKL = volumeDeriCpts (
         degrees, knots, CtrlPnts, deriDepth, r0, spans);

      for (int k = 0; k <= du[0]; k ++) {
         int dd0 = Math.min (deriDepth-k, du[1]);
         for (int l = 0; l <= dd0; l++) {
            int dd1 = Math.min (deriDepth-k-l, du[2]);
            for (int n = 0; n <= dd1; n++) {
               VKD[k][l][n] = new Point3d ();

               for (int kk = 0; kk <= degrees[2] - n; kk++) {
                  Point3d tmp = new Point3d ();

                  for (int j = 0; j <= degrees[1] - l; j++) {
                     Point3d tmp1 = new Point3d ();

                     for (int i = 0; i <= degrees[0] - k; i++) {
                        Point3d tmp2 = new Point3d (PKL[k][l][n][i][j][kk]);
                        tmp2.scale (basis[0][degrees[0]-k][i]);
                        tmp1.add (tmp2);
                     }

                     tmp1.scale (basis[1][degrees[1]-l][j]);
                     tmp.add (tmp1);
                  }

                  tmp.scale (basis[2][degrees[2]-n][kk]);
                  VKD[k][l][n].add (tmp);
               }
               //System.out.println (VKD[k][l][n]);
            }
         }
      }


      return VKD;
   }

   /**
    * compute the control points of all derivative curves up to and including 
    * the <tt>deriDepth-th</tt> derivative <tt>(deriDepth <= p)</tt>; 
    * @param degree 
    * @param knots knots vector
    * @param CtrlPnts  control points
    * @param deriDepth <tt>deriDepth <= degree</tt>
    * @param r0 lower bound of index range
    * @param r1 upper bound of index range
    * @return <tt>PK[k][i]</tt> is the <i>i-th</i> control points of the <i>k-th</i>
    * derivative curves, where <tt>0 <= k <= d </tt> and <tt>r0 <= i <= r1-k</tt>. If
    * <tt>r0 = 0</tt> and <tt>r1 = n</tt>, all control points are computed.
    */
   protected static Point3d [][] curveDeriCpts (int degree, double [] knots, 
      Point3d [] CtrlPnts, int deriDepth, int r0, int r1) {
      if (r1 < r0 || r0 < 0 || r1 < 0) {
         throw new IllegalArgumentException ("Bad index range!");
      }
      if (r1 > CtrlPnts.length) {
         throw new IllegalArgumentException ("Index range is too large!");
      }
      if (deriDepth > degree) {
         throw new IllegalArgumentException ("Derivative depth is too large!");
      }
      if (deriDepth <= 0) {
         throw new IllegalArgumentException ("Derivative depth must be positive!");
      }

      Point3d [][] PK = new Point3d [deriDepth+1][];
      int r = r1 - r0;
      // initialize PK
      for (int k = 0; k <= deriDepth; k++) {
         PK[k] = new Point3d [r-k+1];
      }
      for (int i = 0; i <= r; i++) {
         PK[0][i] = new Point3d (CtrlPnts[r0 + i]);
      }
      for (int k = 1; k <= deriDepth; k++) {
         int tmp = degree - k + 1;
         for (int i = 0; i <= r-k; i++) {
            // P_i+1 - P_i
            PK[k][i] = new Point3d (PK[k-1][i+1]);
            PK[k][i].sub (PK[k-1][i]);
            // U_r0+i+degree+1 - U_r0+i+k
            double knotDif = knots[r0+i+degree+1] - knots[r0+i+k];
            PK[k][i].scale ((double)tmp);
            PK[k][i].scale (1.0/knotDif);
         }
      }
      return PK;
   }

   /**
    * compute the control points of all derivative curves up to and including 
    * the <tt>deriDepth-th</tt> derivative <tt>(deriDepth <= p)</tt>; 
    * @param degree 
    * @param knots knots vector
    * @param CtrlPnts  control points
    * @param deriDepth <tt>deriDepth <= degree</tt>
    * @param r0 lower bound of index range
    * @param r1 upper bound of index range
    * @return <tt>PK[k][i]</tt> is the <i>i-th</i> control points of the <i>k-th</i>
    * derivative curves, where <tt>0 <= k <= d </tt> and <tt>r0 <= i <= r1-k</tt>. If
    * <tt>r0 = 0</tt> and <tt>r1 = n</tt>, all control points are computed.
    */
   protected static Point3d [][][][] surfaceDeriCpts (int [] degrees, double [][] knots, 
      Point3d [][] CtrlPnts, int deriDepth, int [] r0, int [] r1) {
      if (degrees.length != 2) {
         throw new IllegalArgumentException ("Degree dimension must be 2!");
      }
      int sumDegree = degrees[0] + degrees [1];

      if (sumDegree < deriDepth) {
         throw new IllegalArgumentException ("Derivative depth is too large!");
      }


      int [] du = new int [2];
      int [] r = new int [2];
      for (int i = 0; i < 2; i++) {
         du[i] = Math.min (deriDepth, degrees[i]);
         r[i] = r1[i] - r0[i];
      }

      Point3d [][][][] PK = new Point3d [deriDepth+1][deriDepth+1][r1[0]-r0[0]+1][r1[1]-r0[1]+1];

      for (int j = r0[1]; j <= r1[1]; j++) {
         Point3d [] P = new Point3d [CtrlPnts.length];
         for (int i = 0; i < P.length; i++) {
            P[i] = CtrlPnts[i][j];
         }
         Point3d [][] tmp = curveDeriCpts (degrees[0], knots[0], P, du[0], r0[0], r1[0]);
         for (int k = 0; k <= du[0]; k++) {
            for (int i = 0; i <= r[0]-k; i++) {
               PK[k][0][i][j-r0[1]] = tmp[k][i];
            }
         }
      }

      double [] V  = new double [knots[1].length - r0[1]];
      for (int j = 0; j < V.length; j++) {
         V[j] = knots[1][j+r0[1]];
      }
      for (int k = 0; k < du[0]; k++) {
         for (int i=0; i <= r[0]-k; i++) {
            int dd = Math.min (deriDepth - k, du[1]);
            Point3d [][] tmp = curveDeriCpts (degrees[1], V, PK[k][0][i], dd, 0, r[1]);
            for (int l = 1; l <= dd; l++) {
               for (int j = 0; j <= r[1]-l; j++) {
                  PK[k][l][i][j] = tmp[l][j];
               }
            }
         }
      }

      return PK;
   }

   /**
    * compute the control points of all derivative curves up to and including 
    * the <tt>deriDepth-th</tt> derivative <tt>(deriDepth <= p)</tt>; 
    * @param degree 
    * @param knots knots vector
    * @param CtrlPnts  control points
    * @param deriDepth <tt>deriDepth <= degree</tt>
    * @param r0 lower bound of index range
    * @param r1 upper bound of index range
    * @return <tt>PK[k][i]</tt> is the <i>i-th</i> control points of the <i>k-th</i>
    * derivative curves, where <tt>0 <= k <= d </tt> and <tt>r0 <= i <= r1-k</tt>. If
    * <tt>r0 = 0</tt> and <tt>r1 = n</tt>, all control points are computed.
    */
   protected static Point3d [][][][][][] volumeDeriCpts (int [] degrees, double [][] knots, 
      Point3d [][][] CtrlPnts, int deriDepth, int [] r0, int [] r1) {
      if (degrees.length != 3) {
         throw new IllegalArgumentException ("Degree dimension must be 3!");
      }
      int sumDegree = degrees[0] + degrees [1] + degrees [2];
      if (sumDegree < deriDepth) {
         throw new IllegalArgumentException ("Derivative depth is too large!");
      }

      int [] du = new int [3];
      int [] r = new int [3];
      for (int i = 0; i < 3; i++) {
         du[i] = Math.min (deriDepth, degrees[i]);
         r[i] = r1[i] - r0[i];
      }

      Point3d [][][][][][] PK = new Point3d [deriDepth+1][deriDepth+1][deriDepth+1]
      [r1[0] - r0[0] + 1] [r1[1] - r0[1] + 1] [r1[2] - r0[2] + 1];

      int [] degs = {degrees[0], degrees[1]};
      double [][] knts = {knots[0], knots[1]};
      int [] rr0 = {r0[0], r0[1]};
      int [] rr1 = {r1[0], r1[1]};
      for (int kk = r0[2]; kk <= r1[2]; kk++) {
         Point3d [][] P = new Point3d [CtrlPnts.length][];
         for (int i = 0; i < P.length; i++) {
            P[i] = new Point3d[CtrlPnts[i].length];
            for (int j = 0; j < P[i].length; j++) {
               P[i][j] = CtrlPnts[i][j][kk];
            }
         }
         Point3d [][][][] tmp = surfaceDeriCpts (degs, knts, P, Math.max (du[0], du[1]), rr0, rr1);
         for (int k = 0; k <= du[0]; k++) {
            for (int l = 0; l <= du[1]; l++) {
               for (int i = 0; i <= r[0]-k; i++) {
                  for (int j = 0; j <= r[1]-l; j++) {
                     PK[k][l][0][i][j][kk-r0[2]] = tmp[k][l][i][j];
                  }
               }
            }

         }
      }

      double [] W  = new double [knots[2].length - r0[2]];
      for (int kk = 0; kk < W.length; kk++) {
         W[kk] = knots[2][kk+r0[2]];
      }
      for (int k = 0; k < du[0]; k++) {
         for (int i=0; i <= r[0]-k; i++) {
            for (int l = 0; l < du[1]; l++) {
               for (int j = 0; j <= r[1]-l; j++) {

                  int dd = Math.min (deriDepth - k - l, du[2]);
                  Point3d [][] tmp = curveDeriCpts (degrees[2], W, PK[k][l][0][i][j], dd, 0, r[2]);
                  for (int n = 1; n <= dd; n++) {
                     for (int kk = 0; kk <= r[2]-n; kk++) {
                        PK[k][l][n][i][j][kk] = tmp[n][kk];
                     }
                  }
               }
            }
         }
      }

      return PK;
   }






}

