package artisynth.models.swallowingRegistrationTool.utilities;

import java.awt.Color;

import artisynth.core.femmodels.FemElement3d;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.femmodels.HexElement;
import artisynth.core.femmodels.IntegrationData3d;
import artisynth.core.femmodels.IntegrationPoint3d;
import artisynth.core.femmodels.PyramidElement;
import artisynth.core.femmodels.TetElement;
import artisynth.core.femmodels.WedgeElement;
import artisynth.core.mechmodels.MechModel;
import artisynth.models.frank2.frankUtilities.StopWatch;
import artisynth.models.frank2.frankUtilities.StopWatch.Units;
import artisynth.models.modelOrderReduction.SparseBlockMatrix;
import artisynth.models.swallowingRegistrationTool.optimizers.ARUNSVDOptimizer;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.Matrix3d;
import maspack.matrix.Matrix3x3Block;
import maspack.matrix.MatrixBlock;
import maspack.matrix.MatrixNd;
import maspack.matrix.Point3d;
import maspack.matrix.SVDecomposition3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;

/**
 * 
 * @author KeyiTang
 *
 */
public class FEMQualityUtilities {

   static final int[] TetEdgeIdxs = {
                                     2,   0, 1,
                                     2,   0, 2,
                                     2,   0, 3,
                                     2,   1, 2,
                                     2,   2, 3,
                                     2,   3, 1 
   };
   
   static final int [] lineWidth = {1, 1, 2, 3, 5};

   public FEMQualityUtilities () {
      // TODO Auto-generated constructor stub
   }
   
   /*
   public void renderMeanRatioForElements (FemModel3d fem) {
      double [] etas = evalMeanRatios (fem);
      int size = lineWidth.length;
      double unit = 1.0 / (double)size;
      double max = 255.0;
      
      
      for (int i = 0; i < etas.length; i++) {
         double eta = etas[i];
         FemElement3d ele = fem.getElement (i);
         int width, green;
         Color rgb;

         // width
         if (eta == 0) {
            width = 7;
            rgb = new Color (0, 0, 255);
         }
         else {
            width = lineWidth[(int)Math.floor ((1.0-eta)/unit)];
            //System.out.println (width);
            // color
            green = (int)Math.round (eta * max);
            //System.out.println (green);
            rgb = new Color ((int)max - green, green, 0);
         }
         ele.createRenderProps ();
         RenderProps.setLineColor (ele, rgb);
         RenderProps.setLineWidth (ele, width);
      }
   }*/
   
   public void renderMeanRatioForElements (FemModel3d fem, double [] ths) {
      if (ths == null || ths.length == 0) {
         System.out.println ("warning: threshold array is empty!");
         return;
      }
      
      double old = 0;
      for (double th : ths) {
         if (th < old) {
            System.out.println ("warning: invalid threshold values!");
            return;
         }
         old = th;
      }
      
      double [] etas = evalMeanRatios (fem);
      int size = ths.length;
      double unit = 255.0 / (double)size;
      int [] greens = new int [size];
      for (int i = 0; i < size; i++) {
         greens [i]=  (int)((i + 1) * unit);
      }
      
      for (int i = 0; i < etas.length; i++) {
         double eta = etas[i];
         FemElement3d ele = fem.getElement (i);
         int green = 255;
         Color rgb;

         if (eta <= ths [0]) {
            rgb = new Color (0, 0, 255);
         }
         else {
            // color
            for (int j = size - 1; j >= 0; j--) {
               if (eta > ths [j]) {
                  green = greens [j];
                  break;
               }
            }
            rgb = new Color (255 - green, green, 0);
         }
         
         //ele.createRenderProps ();
         RenderProps.setFaceColor (ele, rgb);
      }
   }
   
   public double evalMeanRatio (Face face) {
      double eta = -1;
      if (face.numVertices () == 3) {
         eta = evalTriangleMeanRatio (
            face.getTriVertices ()[0].getPosition (), 
            face.getTriVertices ()[1].getPosition (),
            face.getTriVertices ()[2].getPosition ());
      }
      else if (face.numVertices () == 4) {
         eta = evalQuadMeanRatio (
            face.getVertices ()[0].getPosition (), 
            face.getVertices ()[1].getPosition (),
            face.getVertices ()[2].getPosition (),
            face.getVertices ()[3].getPosition ());
      }
      else {
         System.err.println ("Unknown face type! set mean-ratio value to -1!");
      }
      return eta;
   }
   
   public double [] evalMeanRatios (PolygonalMesh surface) {
      double [] etas = new double [surface.numFaces ()];
      for (int i = 0; i < etas.length; i++) {
         etas [i] = evalMeanRatio (surface.getFace (i));
      }
      return etas;
   }

   public double[] evalMeanRatios (FemModel3d fem) {
      double [] etas = new double [fem.getElements ().size ()];
      for (int i = 0; i < etas.length; i++) {
         double eta = evalMeanRatio (fem.getElement (i));
         etas [i] = eta;
      }
      return etas;
   }
   
   public double evalMeanRatio (FemElement3d ele) {
      if (ele instanceof TetElement) {
         return evalMeanRatio ((TetElement)ele);
      }
      else if (ele instanceof HexElement) {
         return evalMeanRatio((HexElement)ele);
      }
      else if (ele instanceof PyramidElement) {
         return evalMeanRatio ((PyramidElement)ele);
      }
      else if (ele instanceof WedgeElement) {
         return evalMeanRatio ((WedgeElement)ele);
      }
      else {
         throw new IllegalArgumentException ("Incompatible element type!");
      }
   }

   public double evalMeanRatio (TetElement ele) {
      PseudoTet tet = new PseudoTet (ele);
      double eta = tet.computeMeanRatio ();
      return eta;
   }
   
   public double evalMeanRatio (PyramidElement ele) {
      double eta = 0;
      // this is an empirical value
      double sp = 1.123;
      FemNode3d [] nodes = ele.getNodes ();
      if (nodes.length < 5) {
         throw new IllegalArgumentException (
            "Incomplete Pyramid Element!");
      }
      
      double p = evalQuadMeanRatio (
         nodes[0].getPosition (), 
         nodes[1].getPosition (), 
         nodes[2].getPosition (), 
         nodes[3].getPosition ());
      
      if (p == 0){
         return 0;
      }
      
      eta = p * sp;
      double [] mus = new double [4];
      PseudoTet tet = new PseudoTet ();
      tet.setNodes (nodes[0], nodes[1], nodes[2], nodes[4]);
      mus[0] = tet.computeMeanRatio ();
      tet.setNodes (nodes[0], nodes[1], nodes[3], nodes[4]);
      mus[1] = tet.computeMeanRatio ();
      tet.setNodes (nodes[0], nodes[2], nodes[3], nodes[4]);
      mus[2] = tet.computeMeanRatio ();
      tet.setNodes (nodes[1], nodes[2], nodes[3], nodes[4]);
      mus[3] = tet.computeMeanRatio ();
      
      double min = Double.MAX_VALUE;
      for (double mu : mus) {
         if (mu == 0) return 0;
         if (min > mu) min = mu;
      }
      
      eta = eta * min;
      
      eta = Math.sqrt (eta);
      return eta;
   }
   
   public double evalMeanRatio (WedgeElement ele) {
      double eta = 0;
      // this is an empirical value
      double sp = 1.16;
      FemNode3d [] nodes = ele.getNodes ();
      if (nodes.length < 6) {
         throw new IllegalArgumentException (
            "Incomplete Pyramid Element!");
      }
      
      double p = evalQuadMeanRatio (
         nodes[1].getPosition (), 
         nodes[4].getPosition (), 
         nodes[5].getPosition (), 
         nodes[2].getPosition ());
      if (p == 0){
         return 0;
      }
      double tmpp = evalQuadMeanRatio (
         nodes[0].getPosition (), 
         nodes[3].getPosition (), 
         nodes[4].getPosition (), 
         nodes[1].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      tmpp = evalQuadMeanRatio (
         nodes[2].getPosition (), 
         nodes[5].getPosition (), 
         nodes[3].getPosition (), 
         nodes[0].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      
      eta = p * sp;
      double [] mus = new double [6];
      PseudoTet tet = new PseudoTet ();
      tet.setNodes (nodes[0], nodes[1], nodes[2], nodes[3]);
      mus[0] = tet.computeMeanRatio ();
      tet.setNodes (nodes[0], nodes[1], nodes[2], nodes[4]);
      mus[1] = tet.computeMeanRatio ();
      tet.setNodes (nodes[0], nodes[1], nodes[2], nodes[5]);
      mus[2] = tet.computeMeanRatio ();
      tet.setNodes (nodes[0], nodes[3], nodes[4], nodes[5]);
      mus[3] = tet.computeMeanRatio ();
      tet.setNodes (nodes[1], nodes[3], nodes[4], nodes[5]);
      mus[4] = tet.computeMeanRatio ();
      tet.setNodes (nodes[2], nodes[3], nodes[4], nodes[5]);
      mus[5] = tet.computeMeanRatio ();
      
      double min = Double.MAX_VALUE;
      for (double mu : mus) {
         if (mu == 0) return 0;
         if (min > mu) min = mu;
      }
      
      eta = eta * min;
      
      eta = Math.sqrt (eta);
      return eta;
   }
   
   public double evalMeanRatio (HexElement ele) {
      double eta = 0;
      // this is an empirical value
      double sp = 1.19;
      FemNode3d [] nodes = ele.getNodes ();
      if (nodes.length < 8) {
         throw new IllegalArgumentException (
            "Incomplete Pyramid Element!");
      }
      
      double p = evalQuadMeanRatio (
         nodes[0].getPosition (), 
         nodes[4].getPosition (), 
         nodes[5].getPosition (), 
         nodes[1].getPosition ());
      if (p == 0){
         return 0;
      }
      double tmpp = evalQuadMeanRatio (
         nodes[1].getPosition (), 
         nodes[5].getPosition (), 
         nodes[6].getPosition (), 
         nodes[2].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      tmpp = evalQuadMeanRatio (
         nodes[2].getPosition (), 
         nodes[6].getPosition (), 
         nodes[7].getPosition (), 
         nodes[3].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      tmpp = evalQuadMeanRatio (
         nodes[3].getPosition (), 
         nodes[7].getPosition (), 
         nodes[4].getPosition (), 
         nodes[0].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      tmpp = evalQuadMeanRatio (
         nodes[0].getPosition (), 
         nodes[1].getPosition (), 
         nodes[2].getPosition (), 
         nodes[3].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      tmpp = evalQuadMeanRatio (
         nodes[4].getPosition (), 
         nodes[7].getPosition (), 
         nodes[6].getPosition (), 
         nodes[5].getPosition ());
      if (tmpp == 0){
         return 0;
      }
      p = Math.min (p, tmpp);
      
      eta = p * sp;
      
      double [] mus = new double [8];
      PseudoTet tet = new PseudoTet ();
      tet.setNodes (nodes[0], nodes[1], nodes[3], nodes[4]);
      mus[0] = tet.computeMeanRatio ();
      tet.setNodes (nodes[1], nodes[2], nodes[0], nodes[5]);
      mus[1] = tet.computeMeanRatio ();
      tet.setNodes (nodes[2], nodes[3], nodes[1], nodes[6]);
      mus[2] = tet.computeMeanRatio ();
      tet.setNodes (nodes[3], nodes[0], nodes[2], nodes[7]);
      mus[3] = tet.computeMeanRatio ();
      tet.setNodes (nodes[4], nodes[7], nodes[5], nodes[0]);
      mus[4] = tet.computeMeanRatio ();
      tet.setNodes (nodes[5], nodes[4], nodes[6], nodes[1]);
      mus[5] = tet.computeMeanRatio ();
      tet.setNodes (nodes[6], nodes[5], nodes[7], nodes[2]);
      mus[6] = tet.computeMeanRatio ();
      tet.setNodes (nodes[7], nodes[6], nodes[4], nodes[3]);
      mus[7] = tet.computeMeanRatio ();
      
      double min = Double.MAX_VALUE;
      for (double mu : mus) {
         if (mu == 0) return 0;
         if (min > mu) min = mu;
      }

      eta = eta * min;
      
      eta = Math.sqrt (eta);
      return eta;
   }



   /**
    * 
    * @author KeyiTang
    *
    */
   public static class PseudoTet {
      FemNode3d [] myNodes = new FemNode3d [4];
      double myVolume = 0;
      double myMeanRatio = 0;
      
      public PseudoTet () {
         
      }

      public PseudoTet (FemNode3d node0, FemNode3d node1, 
         FemNode3d node2, FemNode3d node3) {
         setNodes (node0, node1, node2, node3);
      }

      public PseudoTet (FemNode3d [] nodes) {
         setNodes(nodes);
      }

      public PseudoTet (TetElement ele) {
         setNodes (ele.getNodes ());
      }

      public void setNodes (FemNode3d node0, FemNode3d node1, 
         FemNode3d node2, FemNode3d node3) {
         myNodes[0] = node0;
         myNodes[1] = node1;
         myNodes[2] = node2;
         myNodes[3] = node3;
      }

      public void setNodes (FemNode3d [] nodes) {
         if (nodes.length < 4) {
            throw new IllegalArgumentException (
            "At least 4 nodes!");
         }

         setNodes (nodes[0], nodes[1], nodes[2], nodes[3]);
      }

      /**
       * 
       * @return
       * For valid elements, mean ratio value is in
       * the range of (0, 1]; 
       * For invalid elements, mean ratio value is 
       * set to zero;
       */
      public double computeMeanRatio () {
         // clockwise for outward normal vector
         computeVolume ();

         // tolerance for flat elements
         if (Math.abs (myVolume) < 1E-16) {
            return 0;
         }
         
         double eta = 12 * Math.pow (3*Math.abs (myVolume), (2.0/3.0));

         double deno = 0;
         int [] indices = FEMQualityUtilities.TetEdgeIdxs;
         int n = 2;
         for (int idxE = 0; idxE < indices.length; idxE += (n+1)) {
            n = indices[idxE];
            Point3d pnt1 = null;
            Point3d pnt2 = null;
            // avoid repeatly regularize on a same edge
            pnt1 = myNodes[indices[idxE+1]].getPosition ();
            pnt2 = myNodes[indices[idxE+2]].getPosition ();
            double l2 = pnt1.distanceSquared (pnt2);
            deno += l2;
         }

         eta /= deno;
         if (eta > 1.0) {
            eta = 1.0;
         }
         myMeanRatio = eta;

         return myMeanRatio;
      }
      
      /**
       * 
       * @return
       * For valid elements, mean ratio value is in
       * the range of (0, 1]; 
       * For invalid elements, mean ratio value is 
       * set to zero;
       */
      public static double computeMeanRatio (TetElement tet) {
         // clockwise for outward normal vector
         double volume = computeVolume (tet);
         FemNode3d [] nodes = tet.getNodes ();

         // tolerance for flat elements
         if (Math.abs (volume) < 1E-16) {
            return 0;
         }
         
         double eta = 12 * Math.pow (3*Math.abs (volume), (2.0/3.0));

         double deno = 0;
         int [] indices = FEMQualityUtilities.TetEdgeIdxs;
         int n = 2;
         for (int idxE = 0; idxE < indices.length; idxE += (n+1)) {
            n = indices[idxE];
            Point3d pnt1 = null;
            Point3d pnt2 = null;
            // avoid repeatly regularize on a same edge
            pnt1 = nodes[indices[idxE+1]].getPosition ();
            pnt2 = nodes[indices[idxE+2]].getPosition ();
            double l2 = pnt1.distanceSquared (pnt2);
            deno += l2;
         }

         eta /= deno;

         return eta;
      }
      
      /**
       * 
       * @return
       * For valid elements, mean ratio value is in
       * the range of (0, 1]; 
       * For invalid elements, mean ratio value is 
       * set to zero;
       */
      public static double computeMeanRatio (
         FemNode3d node0, FemNode3d node1, 
         FemNode3d node2, FemNode3d node3) {
         // clockwise for outward normal vector
         double volume = computeVolume (
            node0, node1, node2, node3);
         FemNode3d [] nodes = {node0, node1,
                               node2, node3};

         // tolerance for flat elements
         if (Math.abs (volume) < 1E-16) {
            return 0;
         }
         
         double eta = 12 * Math.pow (3*Math.abs (volume), (2.0/3.0));

         double deno = 0;
         int [] indices = FEMQualityUtilities.TetEdgeIdxs;
         int n = 2;
         for (int idxE = 0; idxE < indices.length; idxE += (n+1)) {
            n = indices[idxE];
            Point3d pnt1 = null;
            Point3d pnt2 = null;
            pnt1 = nodes[indices[idxE+1]].getPosition ();
            pnt2 = nodes[indices[idxE+2]].getPosition ();
            double l2 = pnt1.distanceSquared (pnt2);
            deno += l2;
         }

         eta /= deno;

         return eta;
      }

      public double computeVolume () {
         myVolume = 
         TetElement.computeVolume (myNodes[0], myNodes[1], 
            myNodes[2], myNodes[3]);
         return myVolume;
      }
      
      public static double computeVolume (TetElement tet) {
         FemNode3d [] nodes = tet.getNodes ();
         double myVolume = 
         TetElement.computeVolume (nodes[0], nodes[1], 
            nodes[2], nodes[3]);
         return myVolume;
      }
      
      public static double computeVolume (
         FemNode3d node0, FemNode3d node1, 
         FemNode3d node2, FemNode3d node3) {
         double myVolume = 
         TetElement.computeVolume (node0, node1, 
            node2, node3);
         return myVolume;
      }

      public double getMeanRatio () {
         return myMeanRatio;
      }

      public double getVolume () {
         return myVolume;
      }
   }

   public static double evalTriangleMeanRatio (Point3d p0, Point3d p1, Point3d p2) {
      double eta = 0;
      
      double d01x = p1.x - p0.x;
      double d01y = p1.y - p0.y;
      double d01z = p1.z - p0.z;

      double d02x = p2.x - p0.x;
      double d02y = p2.y - p0.y;
      double d02z = p2.z - p0.z;
      
      double d12x = p2.x - p1.x;
      double d12y = p2.y - p1.y;
      double d12z = p2.z - p1.z;
      
      // area
      double x = (d01y * d02z - d01z * d02y);
      double y = (d01z * d02x - d01x * d02z);
      double z = (d01x * d02y - d01y * d02x);
      double area = Math.sqrt (x * x + y * y + z * z) / 2;

      // l_01^2 + l_02^2 + l_12^3
      double l01 = d01x*d01x + d01y*d01y + d01z*d01z;
      double l02 = d02x*d02x + d02y*d02y + d02z*d02z;
      double l12 = d12x*d12x + d12y*d12y + d12z*d12z;
      double deno = l01 + l02 + l12;
      
      eta = 4.0 * Math.sqrt (3.0) * area / deno;
      
      if (eta > 1.0) {
         eta = 1.0;
      }
      
      return eta;
   }
   
   /**
    * 4 vertices have been arranged in circular order;
    * For FEM analysis, quadrilateral must be non-degenerated 
    * and simple(i.e. 4 vertices are distinct and 4 edges do 
    * not intersect except at common endpoints) and strictly 
    * convex(i.e. all 4 interior angles are < 180 degrees); 
    * @param p0
    * @param p1
    * @param p2
    * @param p3
    * @return if quadrilateral is not non-degenerated or not
    * strictly convex, return zero;For valid quadrilateral,
    * most commonly return value is in the range of (0, 1], but 
    * currently the maximum value is unknown.
    */
   public static double evalQuadMeanRatio (
      Point3d p0, Point3d p1, Point3d p2, Point3d p3) {
      double eta = 0;
      double sq = 2.0 / Math.sqrt (3);
      
      double p = isQuadrilateralValid (
         p0, p1, p2, p3);
      if (p == 0) {
         return 0;
      }
      
      eta = sq * p;
      
      double mu = Double.MAX_VALUE;
      mu = Math.min (mu, evalTriangleMeanRatio (p0, p1, p2));
      mu = Math.min (mu, evalTriangleMeanRatio (p0, p1, p3));
      mu = Math.min (mu, evalTriangleMeanRatio (p0, p2, p3));
      mu = Math.min (mu, evalTriangleMeanRatio (p1, p2, p3));
      
      eta *= mu;
      
      eta = Math.sqrt (eta);

      return eta;
   }
   
   /**
    * 4 vertices have been arranged in circular order;
    * For FEM analysis, quadrilateral must be non-degenerated 
    * and simple(i.e. 4 vertices are distinct and 4 edges do 
    * not intersect except at common endpoints) and strictly 
    * convex(i.e. all 4 interior angles are < 180 degrees); 
    * Otherwise the quadrilateral is invalid;
    * @param p0
    * @param p1
    * @param p2
    * @param p3
    * @return if quadrilateral is not non-degenerated or not
    * strictly convex, return zero, otherwise return value is 
    * in the range of (0, 1]; value of 1 only for planer square;
    */
   public static double isQuadrilateralValid (
      Point3d p0, Point3d p1, Point3d p2, Point3d p3) {
      Vector3d E01 = new Vector3d ();
      Vector3d E02 = new Vector3d ();
      Vector3d E03 = new Vector3d ();
      Vector3d E12 = new Vector3d ();
      Vector3d E13 = new Vector3d ();
      
      E01.sub (p1, p0);
      E02.sub (p2, p0);
      E03.sub (p3, p0);
      E12.sub (p2, p1);
      E13.sub (p3, p1);
      
      Vector3d N012 = new Vector3d ();
      Vector3d N013 = new Vector3d ();
      Vector3d N023 = new Vector3d ();
      Vector3d N123 = new Vector3d ();
      
      N012.cross (E01, E02);
      N013.cross (E01, E03);
      N023.cross (E02, E03);
      N123.cross (E12, E13);
      N012.normalize ();
      N013.normalize ();
      N023.normalize ();
      N123.normalize ();
      
      double [] ps = {
                      N012.dot (N013),
                      N012.dot (N023),
                      N012.dot (N123),
                      N013.dot (N023),
                      N013.dot (N123),
                      N023.dot (N123)};
      double min = Double.MAX_VALUE;
      for (int i=  0; i < 6; i++) {

         if (min > ps[i]) {
            min = ps[i];
         }
         if (min < 0) return 0;
      }
      
      if (min > 1.0) {
         min = 1.0;
      }
      double p = Math.acos (min);
      p = 1.0 - (2.0/Math.PI) * p;

      return p;
   }
   
   public static Point3d [] findIdealElement (FemElement3d ele) {
      if (ele instanceof TetElement) {
         return findIdealElement ((TetElement)ele);
      }
      else if (ele instanceof HexElement) {
         return findIdealElement((HexElement)ele);
      }
      else if (ele instanceof PyramidElement) {
         return findIdealElement ((PyramidElement)ele);
      }
      else if (ele instanceof WedgeElement) {
         return findIdealElement ((WedgeElement)ele);
      }
      else {
         throw new IllegalArgumentException ("Incompatible element type!");
      }
   }
   
   public static Point3d [] findIdealElement1 (TetElement ele) {
      Point3d [] ideals = new Point3d [4];
      FemNode3d [] nodes = ele.getNodes ();
      double area1 = triangleArea (nodes[0].getPosition (), 
         nodes[1].getPosition (), nodes[2].getPosition ());
      double area2 = triangleArea (nodes[1].getPosition (), 
         nodes[2].getPosition (), nodes[4].getPosition ());
      double area3 = triangleArea (nodes[0].getPosition (), 
         nodes[1].getPosition (), nodes[4].getPosition ());
      double area4 = triangleArea (nodes[0].getPosition (), 
         nodes[2].getPosition (), nodes[4].getPosition ());
      double area = (area1 + area2 + area3 + area4) / 4.0;
      
      double ratio = Math.sqrt (area) / Math.sqrt (Math.sqrt (3.0)/4.0);
      ideals[0] = new Point3d (0, 0, 0);
      ideals[2] = new Point3d (ratio, 0, 0);
      ideals[1] = new Point3d (ratio*0.5, Math.sqrt (3.0)*ratio/2.0, 0);
      ideals[3] = new Point3d (ratio*0.5, Math.sqrt (3.0)*ratio/6.0, Math.sqrt (6.0)*ratio/3.0);
      
      return ideals;
   }
   
public static SparseBlockMatrix computeIdealElementNiNj (FemElement3d ele) {
      
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

         // integration weight
         double dv = igp.getWeight ();
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
   
   
   public static Point3d [] findIdealElement (TetElement ele) {
      Point3d [] ideals = new Point3d [4];
      FemNode3d [] nodes = ele.getNodes ();
      
      MatrixNd DataP = new MatrixNd (4, 3);
      MatrixNd DataI = new MatrixNd (4, 3);
      
      for (int i = 0; i < 4; i++) {
         for (int j = 0; j < 3; j++) {
            DataP.set (i, j, nodes[i].getPosition ().get (j));  
         }
      }
      
      DataI.setRow (0, new double [3]);
      DataI.setRow (1, new double [] {1.0, 0.0, 0.0});
      DataI.setRow (2, new double [] {0.5, Math.sqrt (3.0)/2.0, 0.0});
      DataI.setRow (3, new double [] {0.5, Math.sqrt (3.0)/6.0, Math.sqrt (6.0)/3.0});
      /*
      double ratio = getScaleFactor (DataI, DataP);
      ideals[0] = new Point3d (0, 0, 0);
      ideals[2] = new Point3d (ratio, 0, 0);
      ideals[1] = new Point3d (ratio*0.5, Math.sqrt (3.0)*ratio/2.0, 0);
      ideals[3] = new Point3d (ratio*0.5, Math.sqrt (3.0)*ratio/6.0, Math.sqrt (6.0)*ratio/3.0);*/
      AffineTransform3d affine = ARUNSVDOptimizer.fitRigid (
         DataI, DataP/*, computeIdealElementNiNj(ele)*/, true);
      for (int i = 0; i < 4; i++) {
         ideals[i] = new Point3d ();
         DataI.getRow (i, ideals[i]);
         ideals[i].transform (affine);
      }
      
      return ideals;
   }
   
   public static Point3d [] findIdealElement (PyramidElement ele) {
      Point3d [] ideals = new Point3d [5];
      FemNode3d [] nodes = ele.getNodes ();
      
      MatrixNd DataP = new MatrixNd (5, 3);
      MatrixNd DataI = new MatrixNd (5, 3);
      
      for (int i = 0; i < 5; i++) {
         for (int j = 0; j < 3; j++) {
            DataP.set (i, j, nodes[i].getPosition ().get (j));  
         }
      }
      
      DataI.setRow (0, new double [3]);
      DataI.setRow (3, new double [] {1.0, 0.0, 0.0});
      DataI.setRow (2, new double [] {1.0, 1.0, 0.0});
      DataI.setRow (1, new double [] {0.0, 1.0, 0.0});
      DataI.setRow (4, new double [] {0.5, 0.5, -0.975});
      
      AffineTransform3d affine = ARUNSVDOptimizer.fitRigid (
         DataI, DataP/*, computeIdealElementNiNj(ele)*/, true);
      
      for (int i = 0; i < 5; i++) {
         ideals[i] = new Point3d ();
         DataI.getRow (i, ideals[i]);
         ideals[i].transform (affine);
      }
      
      return ideals;
   }
   
   public static Point3d [] findIdealElement (WedgeElement ele) {
      Point3d [] ideals = new Point3d [6];
      FemNode3d [] nodes = ele.getNodes ();
      
      MatrixNd DataP = new MatrixNd (6, 3);
      MatrixNd DataI = new MatrixNd (6, 3);
      
      for (int i = 0; i < 6; i++) {
         for (int j = 0; j < 3; j++) {
            DataP.set (i, j, nodes[i].getPosition ().get (j));  
         }
      }
      
      DataI.setRow (0, new double [3]);
      DataI.setRow (2, new double [] {1.0, 0.0, 0.0});
      DataI.setRow (1, new double [] {0.5, Math.sqrt (3.0)/2.0, 0.0});
      DataI.setRow (3, new double [] {0.0, 0.0, -1.0});
      DataI.setRow (5, new double [] {1.0, 0.0, -1.0});
      DataI.setRow (4, new double [] {0.5, Math.sqrt (3.0)/2.0, -1.0});
      
      AffineTransform3d affine = ARUNSVDOptimizer.fitRigid (
         DataI, DataP/*, computeIdealElementNiNj(ele)*/, true);
      
      for (int i = 0; i < 6; i++) {
         ideals[i] = new Point3d ();
         DataI.getRow (i, ideals[i]);
         ideals[i].transform (affine);
      }
      
      return ideals;
   }
   
   public static Point3d [] findIdealElement (HexElement ele) {
      Point3d [] ideals = new Point3d [8];
      FemNode3d [] nodes = ele.getNodes ();
      
      MatrixNd DataP = new MatrixNd (8, 3);
      MatrixNd DataI = new MatrixNd (8, 3);
      
      for (int i = 0; i < 8; i++) {
         for (int j = 0; j < 3; j++) {
            DataP.set (i, j, nodes[i].getPosition ().get (j));  
         }
      }
      
      DataI.setRow (4, new double [3]);
      DataI.setRow (7, new double [] {0.0, 1.0, 0.0});
      DataI.setRow (6, new double [] {1.0, 1.0, 0.0});
      DataI.setRow (5, new double [] {1.0, 0.0, 0.0});
      DataI.setRow (0, new double [] {0.0, 0.0, 1.0});
      DataI.setRow (3, new double [] {0.0, 1.0, 1.0});
      DataI.setRow (2, new double [] {1.0, 1.0, 1.0});
      DataI.setRow (1, new double [] {1.0, 0.0, 1.0});
      
      AffineTransform3d affine = ARUNSVDOptimizer.fitRigid (
         DataI, DataP/*, computeIdealElementNiNj(ele)*/, true);
      
      for (int i = 0; i < 8; i++) {
         ideals[i] = new Point3d ();
         DataI.getRow (i, ideals[i]);
         ideals[i].transform (affine);
      }
      
      return ideals;
   }
   
   private static double getScaleFactor (MatrixNd SrcData, MatrixNd TgtData) {
      MatrixNd CenDataSrc = new MatrixNd ();
      MatrixNd CenDataTgt = new MatrixNd ();
      PCA.centralizeData (CenDataSrc, SrcData);
      PCA.centralizeData (CenDataTgt, TgtData);
      
      double sq = 0;
      double [] bufs = CenDataSrc.getBuffer ();
      for (double buf : bufs) {
         sq += buf * buf;
      }
      sq /= CenDataSrc.rowSize ();
      
      // CenDataTgt^T x CenDataSrc
      Matrix3d Cov =  new Matrix3d ();
      for (int i = 0; i < CenDataTgt.colSize (); i++) {
         for (int j = 0; j < CenDataSrc.colSize (); j++) {
            double val = 0;
            for (int k = 0; k < CenDataTgt.rowSize (); k++) {
               val += CenDataTgt.get (k, i) * CenDataSrc.get (k, j);
            }
            Cov.set (i, j, val);
         }
      }
      Cov.scale (1.0 / (double)CenDataSrc.rowSize ());
      
      SVDecomposition3d svd = new SVDecomposition3d ();
      svd.factor (Cov);
      Vector3d sig = new Vector3d ();
      svd.getS (sig);
      
      double ratio = (sig.x + sig.y + sig.z) / sq;
      return ratio;
   }
  
   
   /**
    * For testing
    * @param args
    */
   public static void main (String [] args) {
      
      StopWatch sw = new StopWatch ("Main", Units.seconds);
      // test for triangle
      Point3d p0 = new Point3d (0.0, 0.0, 0.0);
      Point3d p1 = new Point3d (1.0, 0.0, 0.0);
      Point3d p2 = new Point3d (0.5, Math.sqrt (3.0)/2.0, 0.0);
      double ratio;
      sw.checkpoint ("triangle start!");
      ratio = evalTriangleMeanRatio (p0, p1, p2);
      sw.checkpoint ("triangle end!");
      sw.checkpoint ("triangle start!");
      ratio = evalTriangleMeanRatio (p0, p1, p2);
      sw.checkpoint ("triangle end!");
      System.out.println (ratio);
      
      // test for quadrilateral
      p2 = new Point3d (1.0, 1.0, 0.0);
      Point3d p3 = new Point3d (0.0, 1.0, 0.0);
      sw.checkpoint ("quadrilateral start!");
      ratio = evalQuadMeanRatio (p0, p1, p2, p3);
      sw.checkpoint ("quadrilateral end!");
      System.out.println (ratio);
      
      // test for pyramid 
      Point3d p4 = new Point3d (0.5, 0.5, 0.975);
      FEMQualityUtilities tool = new FEMQualityUtilities ();
      PyramidElement pele = new PyramidElement (
         new FemNode3d (p0), new FemNode3d (p1),
         new FemNode3d (p2), new FemNode3d (p3), 
         new FemNode3d (p4));
      sw.checkpoint ("pyramid start!");
      ratio = tool.evalMeanRatio (pele);
      sw.checkpoint ("pyramid end!");
      System.out.println (ratio);
      sw.checkpoint ("find ideal pyramid start!");
      Point3d [] ideals = findIdealElement (pele);
      PyramidElement pyramid = new PyramidElement (
         new FemNode3d (p0), new FemNode3d (p1),
         new FemNode3d (p2), new FemNode3d (p3), 
         new FemNode3d (p4));
      pyramid.computeRestVolumes ();
      sw.checkpoint ("find ideal pyramid end!");
      for (int i = 0; i < ideals.length; i++) {
         System.out.println (ideals[i]);
      }
      
      
      // test for wedge
      double z = -1.0;
      p2.set (0.5, Math.sqrt (3.0)/2.0, 0.0);
      p3.set (0.0, 0.0, z);
      p4.set (1.0, 0.0, z);
      Point3d p5 = new Point3d (0.5, Math.sqrt (3.0)/2.0, z);
      WedgeElement wele = new WedgeElement (
         new FemNode3d (p0), new FemNode3d (p2),
         new FemNode3d (p1), new FemNode3d (p3), 
         new FemNode3d (p5), new FemNode3d (p4));
      sw.checkpoint ("wedge start!");
      ratio = tool.evalMeanRatio (wele);
      sw.checkpoint ("wedge end!");
      System.out.println (ratio);
      sw.checkpoint ("find ideal wedge start!");
      ideals = findIdealElement (wele);
      WedgeElement wedge = new WedgeElement (
         new FemNode3d (ideals[0]), new FemNode3d (ideals[1]),
         new FemNode3d (ideals[2]), new FemNode3d (ideals[3]), 
         new FemNode3d (ideals[4]), new FemNode3d (ideals[5]));
      wedge.computeRestVolumes ();
      sw.checkpoint ("find ideal wedge end!");
      for (int i = 0; i < ideals.length; i++) {
         System.out.println (ideals[i] + "-------" + wele.getPoint (i));
      }
      
      // test for hex
      p2.set (1.0, 1.0, 0.0);
      p3.set (0.0, 1.0, 0.0);
      p4.set (0.0, 0.0, 1.0);
      p5.set (1.0, 0.0, 1.0);
      Point3d p6 = new Point3d (1.0, 1.0, 1.0);
      Point3d p7 = new Point3d (0.0, 1.0, 1.0);
      HexElement hele = new HexElement (
         new FemNode3d (p0), new FemNode3d (p3),
         new FemNode3d (p2), new FemNode3d (p1), 
         new FemNode3d (p4), new FemNode3d (p7),
         new FemNode3d (p6), new FemNode3d (p5) );
      sw.checkpoint ("hex start!");
      ratio = tool.evalMeanRatio (hele);
      sw.checkpoint ("hex end!");
      System.out.println (ratio);
      sw.checkpoint ("find ideal hex start!");
      ideals = findIdealElement (hele);
      sw.checkpoint ("find ideal hex end!");
      for (int i = 0; i < 8; i++) {
         System.out.println (ideals[i]);
      }

      // test for tet
      p0.set (0.5 , -0.5, 0.5);
      p1.set (0.5, 0.5, -0.5);
      p2.set (-0.5, -0.5, -0.5);
      p3.set (-0.5, 0.5, 0.5);
      TetElement tele = new TetElement (
         new FemNode3d (p0), new FemNode3d (p1),
         new FemNode3d (p2), new FemNode3d (p3));
      sw.checkpoint ("tet start!");
      ratio = tool.evalMeanRatio (tele);
      sw.checkpoint ("tet end!");
      System.out.println (ratio);
      sw.checkpoint ("find ideal tet start!");
      ideals = findIdealElement (tele);
      TetElement tet = new TetElement (
         new FemNode3d (ideals[0]), new FemNode3d (ideals[1]),
         new FemNode3d (ideals[2]), new FemNode3d (ideals[3]));
      tet.computeRestVolumes ();
      System.out.println (tool.evalMeanRatio (tet));
      sw.checkpoint ("find ideal tet end!");
      for (int i = 0; i < ideals.length; i++) {
         System.out.println (ideals[i] + "-------" + tele.getPoint (i));
      }
      
      // test for hybrid elements
      
   }
   

   public static double triangleArea (
      Point3d p0, Point3d p1, Point3d p2) {
      double d1x = p1.x - p0.x;
      double d1y = p1.y - p0.y;
      double d1z = p1.z - p0.z;

      double d2x = p2.x - p0.x;
      double d2y = p2.y - p0.y;
      double d2z = p2.z - p0.z;

      double x = (d1y * d2z - d1z * d2y);
      double y = (d1z * d2x - d1x * d2z);
      double z = (d1x * d2y - d1y * d2x);

      return Math.sqrt (x * x + y * y + z * z) / 2;
   }
}
