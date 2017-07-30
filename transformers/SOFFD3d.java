package artisynth.models.swallowingRegistrationTool.transformers;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

import javax.swing.JSeparator;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.Point;
import artisynth.core.modelbase.ControllerBase;
import artisynth.core.workspace.RootModel;
import artisynth.models.swallowingRegistrationTool.utilities.MeshModelAgent;
import maspack.geometry.BVFeatureQuery;
import maspack.geometry.BVNode;
import maspack.geometry.BVTree;
import maspack.geometry.Face;
import maspack.geometry.Feature;
import maspack.geometry.LineSegment;
import maspack.geometry.MeshBase;
import maspack.geometry.PointMesh;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.PolylineMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AffineTransform3d;
import maspack.matrix.ImproperStateException;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector2d;
import maspack.matrix.Vector3d;
import maspack.properties.PropertyList;

/**
 * This class is an implementation of surface-oriented FFD, 
 * which is loosely based on reference:
 * <i> Skinning Characters using Surface-Oriented Free-Form Deformations;
 * Karan Singh, Evangelos Kokkevis </i>;
 * @author KeyiTang
 */
public class SOFFD3d extends ControllerBase{

   private PolygonalMesh refMesh;
   private PolygonalMesh undeformedMesh;
   private double baseWeight = DEFAULT_BASE_WEIGHT;
   protected static final double DEFAULT_BASE_WEIGHT = 0.25;
   protected double rangeRatio = 0.1;

   protected ArrayList <Point> slavePoints = new ArrayList <Point> ();
   private HashMap <Point, LinkedList<Face>> oldSpRfsMap = new HashMap <Point, LinkedList<Face>> ();
   private HashMap <Point, double[]> weightsMap = new HashMap <Point, double[]> ();
   private HashMap <Point, LinkedList<Face>> currentSpRfsMap = new HashMap <Point, LinkedList<Face>> ();
   private HashMap <Point, Point3d> cur2undMap = new HashMap <Point, Point3d> ();

   private BasePositionMethod myBaseComputeMethod = BasePositionMethod.Null;
   private SOFFDWeightsMaker myWeightsMaker = null;

   private ControlPanel myPanel;

   public enum BasePositionMethod {
      Null,
      Fixed,
      Rigid,
      ScaledRigid,
      Affine,
   }
  
   
   /**
    * set the face picking and weights computing method
    * @param maker if <tt>maker</tt> is null then use default method;
    * otherwise use the method defined in <tt>maker</tt>.
    */
   public void setWeightsMaker (SOFFDWeightsMaker maker) {
      myWeightsMaker = maker;
   }
  

   // ------------------------- properties ------------------------------//
   static public PropertyList myProps = new PropertyList (SOFFD3d.class, ControllerBase.class);
   @Override
   public PropertyList getAllPropertyInfo() {
      return myProps;
   }

   static {
      myProps.add ("baseMethod", "Base Compute Method", BasePositionMethod.Rigid);
      myProps.add ("baseWeight", "Base weight", 0);
      //myProps.add ("rangeRatio", "Face to point distance range", 0.1);
   }

   public void setBaseMethod (BasePositionMethod method) {
      if (myBaseComputeMethod != method && method == BasePositionMethod.Null) {
         changeBaseWeight (0);
      }
      else if (myBaseComputeMethod == BasePositionMethod.Null
      && method != myBaseComputeMethod) {
         changeBaseWeight (baseWeight);
      }
      myBaseComputeMethod = method;
   }

   public BasePositionMethod getBaseMethod () {
      return myBaseComputeMethod;
   }

   public void setBaseWeight (double weight) {
      if (myBaseComputeMethod != BasePositionMethod.Null) {
         if (weight != baseWeight) {
            changeBaseWeight (weight);
            baseWeight = weight;
         }
      }
   }

   public double getBaseWeight () {
      if (myBaseComputeMethod == BasePositionMethod.Null) {
         return 0;
      }
      return baseWeight;
   }

   public void createControlPanel (RootModel root) {
      myPanel = new ControlPanel ();
      myPanel.addLabel ("Surface-Oriented FFD");
      myPanel.addWidget ("Base Compute Method", this, "baseMethod");
      myPanel.addWidget ("Base Weight", this, "baseWeight");
   }

   public void addSOFFDWidgetsToControlPanel (ControlPanel panel) {
      panel.addWidget (new JSeparator());
      panel.addLabel ("Surface-Oriented FFD");
      panel.addWidget ("Base Compute Method", this, "baseMethod");
      panel.addWidget ("Base Weight", this, "baseWeight");
   }

   public SOFFD3d (PolygonalMesh mesh) {
      setReferenceMesh (mesh);
   }

   /**
    * Reference mesh which is characterize it as a skin mesh 
    * which is used to drive surrounding slave points;
    * @param mesh reference mesh to set, it has to be a pure 
    * triangular mesh, otherwise throw a new input exception. 
    */
   public void setReferenceMesh (PolygonalMesh mesh) {
      if (!mesh.isTriangular ()) {
         throw new IllegalArgumentException (
         "Reference mesh is not pure triangular mesh");
      }
      mesh.autoGenerateNormals ();
      refMesh = mesh;
   }


   public PolygonalMesh getReferenceMesh () {
      return refMesh;
   }

   public void addSlavePoints (Collection<Point> slaves) {
      for (Point pnt : slaves) {
         if (!slavePoints.contains (pnt))
            slavePoints.add (pnt);
      }
   }

   public void getSlavePoints (LinkedList<Point> list) {
      list.addAll (slavePoints);
   }


   public void updateSlavePoints () {
      BasePositionMethod method = myBaseComputeMethod;
      for (Point pnt : currentSpRfsMap.keySet ()) {
         Point3d newPos = new Point3d ();
         double [] ws = weightsMap.get (pnt);
         LinkedList<Face> oldFs = oldSpRfsMap.get (pnt);
         LinkedList<Face> fs = currentSpRfsMap.get (pnt);
         boolean isNP = false;
         if (ws.length == oldFs.size ()) {
            isNP = true;
         }
         for (int i =0 ; i < fs.size (); i++) {
            AffineTransform3d invD = new AffineTransform3d ();
            invD.mulInverseRight (computeFacePosInWorld (fs.get (i)), 
               computeFacePosInWorld (oldFs.get (i)));
            Point3d pos = new Point3d (cur2undMap.get (pnt));
            pos.transform (invD);
            pos.scale (ws[i]);
            newPos.add (pos);
         }
         if (!isNP) {
            Point3d base = new Point3d  ();
            base.set (cur2undMap.get (pnt));

            if (method != BasePositionMethod.Fixed && 
            method != BasePositionMethod.Null) {
               AffineTransform3d Xcu = new AffineTransform3d ();
               ArrayList <Point3d> src = makeMeshPointList (undeformedMesh);
               ArrayList <Point3d> tgt = makeMeshPointList (refMesh);
               if (method == BasePositionMethod.Rigid) {
                  Xcu.fitRigid (tgt, src, false);
               }
               else if (method == BasePositionMethod.ScaledRigid) {
                  Xcu.fitRigid (tgt, src, true);
               }
               else if (method == BasePositionMethod.Affine) {
                  Xcu.fit (tgt, src);
               }
               base.transform (Xcu);
               base.scale (ws[ws.length-1]);
            }
            else if (method == BasePositionMethod.Fixed) {
               base.scale (ws[ws.length-1]);
            }
            else {
               base.set (0, 0, 0);
            }
            newPos.add (base);
         }
         pnt.setPosition (newPos);
      }
   }

   private ArrayList <Point3d> makeMeshPointList (MeshBase mesh) {
      ArrayList <Point3d> list = new ArrayList <Point3d> ();
      for (Vertex3d vtx: mesh.getVertices ()) {
         Point3d pnt = new Point3d ();
         vtx.getWorldPoint (pnt);
         list.add (pnt);
      }
      return list;
   }

   //TODO: change base weight
   //TODO: no base
   private double dmin = Double.MAX_VALUE;
   public void updateMaps () {
      undeformedMesh = (PolygonalMesh)
      MeshModelAgent.makeMeshAgent (refMesh);
      
      if (slavePoints.size () == 0) {
         System.err.println ("No slave points!");
      }
      oldSpRfsMap.clear ();
      currentSpRfsMap.clear ();
      cur2undMap.clear ();

      Point3d near  = new Point3d ();
      for (Point pnt : slavePoints) {
         BVFeatureQuery.getNearestFaceToPoint (
            near, null, undeformedMesh, pnt.getPosition ());
         double nearDis = near.distance (pnt.getPosition ());
         if (nearDis < dmin) {
            dmin = nearDis;
         }
      }

      if (myWeightsMaker == null) {
         for (Point pnt : slavePoints) {
            pickFaceInRange (oldSpRfsMap, pnt, undeformedMesh);
            pickFaceInRange (currentSpRfsMap, pnt, refMesh);
         }
         for (Point pnt: oldSpRfsMap.keySet ()) {
            cur2undMap.put (pnt, new Point3d (pnt.getPosition ()));
         }
         //TODO: just for attention
         computeWeights ();
         if (myBaseComputeMethod == BasePositionMethod.Null) {
            changeBaseWeight (0);
         }
      }
      else {
          LinkedHashMap<List<Face>, double []> map = 
          myWeightsMaker.makeReferenceAndWeights (slavePoints, undeformedMesh);
          
          Set ens = map.entrySet ();
          Iterator it=  ens.iterator ();
          
          int idx = 0;
          while (it.hasNext ()) {
             Entry<List<Face>, double []> me = 
             (Entry<List<Face>, double []>)it.next ();
             LinkedList<Face> oldFaces = new LinkedList<Face>();
             LinkedList<Face> refFaces = new LinkedList<Face>();
             for (Face face : me.getKey ()) {
                oldFaces.add (face);
                refFaces.add (refMesh.getFace (face.getIndex ()));
             }
             oldSpRfsMap.put (slavePoints.get (idx), oldFaces);
             currentSpRfsMap.put (slavePoints.get (idx), refFaces);
             weightsMap.put (slavePoints.get (idx), me.getValue ());
             idx ++;
          }
          
          for (Point pnt: oldSpRfsMap.keySet ()) {
             cur2undMap.put (pnt, new Point3d (pnt.getPosition ()));
          }
          
          //if (myBaseComputeMethod == BasePositionMethod.Null) {
            // changeBaseWeight (0);
          //}
      }
      
   }

   //Xfw
   private AffineTransform3d computeFacePosInWorld (Face face) {
      if (face.isTriangle ()) {
         Vertex3d [] vtxs = face.getTriVertices ();
         Vector3d v1 = new Vector3d ();
         Vector3d v2 = new Vector3d ();
         Vector3d v3 = new Vector3d ();
         Point3d p1 = vtxs[0].getWorldPoint ();
         Point3d p2 = vtxs[1].getWorldPoint ();
         Point3d p3 = vtxs[2].getWorldPoint ();
         v1.sub (p2, p1);
         v2.sub (p3, p1);
         v3.cross (v1, v2);
         v3.normalize ();
         AffineTransform3d X = new AffineTransform3d ();
         X.A.setColumns (v1, v2, v3);
         X.p.set (p1);
         return X;
      }
      else {
         return null;
      }
   }

   private HashMap <Point, Double> scaleMap = new HashMap <Point, Double> ();
   private void computeWeights () {
      weightsMap.clear ();
      scaleMap.clear ();

      for (Point pnt : oldSpRfsMap.keySet ()) {
         LinkedList<Face> faces = oldSpRfsMap.get (pnt);
         Point3d localPnt = new Point3d (cur2undMap.get (pnt));
         localPnt.inverseTransform (
            faces.get (0).getMesh ().getMeshToWorld ());
         // check for near field faces
         boolean [] isNear = new boolean [faces.size ()];
         boolean nearPnt = false;
         int idx = 0;
         for (Face face : faces) {
            Vertex3d [] vtxs = face.getTriVertices ();
            Vector3d r1 = new Vector3d ();
            Vector3d r2 = new Vector3d ();
            Vector3d r3 = new Vector3d ();
            Point3d p1 = vtxs[0].getPosition ();
            Point3d p2 = vtxs[1].getPosition ();
            Point3d p3 = vtxs[2].getPosition ();
            r1.sub (p1, localPnt);
            r2.sub (p2, localPnt);
            r3.sub (p3, localPnt);
            isNear [idx] = checkForNearPair (r1, r2, r3);
            if (isNear[idx]) nearPnt = true;
            idx ++;
         }
         // remove faces in far-field
         double [] weights;
         if (nearPnt) {
            LinkedList <Face> cfs = currentSpRfsMap.get (pnt);
            idx = 0;
            for (boolean isnear :  isNear) {
               if (!isnear) {
                  faces.remove (idx);
                  cfs.remove (idx);
                  idx--;
               }
               idx++;
            }
            // no weight for base position preserving
            weights = new double [faces.size ()];
         }
         else {
            // one more weight for base position preserving
            weights = new double [faces.size ()+1];
         }
         double sumW = 0;
         // start weight computing
         if (nearPnt) {
            // for skin points
            idx = 0;
            for (Face face : faces) {
               Point3d loc = new Point3d ();
               Vector2d coords = new Vector2d ();
               face.computeCoords (localPnt, coords);
               face.computePoint (loc, coords);
               double w = weightFunc (loc, localPnt);
               weights[idx++] = w;
               sumW += w;
            }
         }
         else {
            // for non-skin points
            // feasible for linear interpolation approximation
            // integration of inverse-squrare-distance force
            // profile over triangular domain equal to:
            // area * (f1 + f2 + f3) /3, 
            // where f_i = 1 / r_i^2;
            idx = 0;
            for (Face face : faces) {
               Vertex3d [] vtxs = face.getTriVertices ();
               Vector3d r1 = new Vector3d ();
               Vector3d r2 = new Vector3d ();
               Vector3d r3 = new Vector3d ();
               Point3d p1 = vtxs[0].getPosition ();
               Point3d p2 = vtxs[1].getPosition ();
               Point3d p3 = vtxs[2].getPosition ();
               r1.sub (p1, localPnt);
               r2.sub (p2, localPnt);
               r3.sub (p3, localPnt);
               // force profile
               double [] fp = new double [3];
               fp[0] = r1.normSquared ();
               fp[1] = r2.normSquared ();
               fp[2] = r3.normSquared ();
               double area = face.computeArea ();
               for (double ff : fp) {
                  weights[idx] += 1.0/ff;
               }
               weights[idx] /= 3.0;
               weights[idx] *= area;
               sumW += weights[idx];;
               idx++;
            }
            weights[faces.size ()] = baseWeight;
            sumW += baseWeight;
         }
         // normalize
         for (int i = 0; i < weights.length; i++) {
            weights[i] /= sumW;
         }
         weightsMap.put (pnt, weights);
         scaleMap.put (pnt, new Double (sumW));
      }
   }

   private void changeBaseWeight (double weight) {
      for (Point pnt : weightsMap.keySet ()) {
         double [] weis = weightsMap.get (pnt);
         if (oldSpRfsMap.get (pnt).size () == weis.length) {
            continue;
         }
         double bW = weis[weis.length-1];
         double scale = scaleMap.get (pnt);
         double oldSumW = scale;
         double newSumW = oldSumW - bW*scale + weight;
         double ratio = oldSumW / newSumW;
         for (int i = 0; i < weis.length-1; i++) {
            weis[i] *= ratio;
         }
         weis[weis.length-1] = weight/newSumW;
         scaleMap.put (pnt, new Double (newSumW));
      }
   }

   /*
    * any dot product between any two of vectors 
    * is negative return true, otherwise return false;
    */
   private boolean checkForNearPair (
      Vector3d r1, Vector3d r2, Vector3d r3) {
      if (r1.dot (r2) < 0) return true;
      else if (r2.dot (r3) < 0) return true;
      else if (r3.dot (r1) < 0) return true;
      else return false;
   }


   /**
    * retrieve faces in range based on <tt>rangeRatio</tt>; Add
    * point to faces map into <tt>SRMap</tt>; If no face found, 
    * do nothing.
    * @param SRMap source point --->> reference face map 
    * @param pnt source point
    */
   private void pickFaceInRange (HashMap <Point, LinkedList <Face>> SRMap, Point pnt, PolygonalMesh mesh) {
      Point3d near = new Point3d ();
      BVFeatureQuery.getNearestFaceToPoint (
         near, null, undeformedMesh, pnt.getPosition ());
      double nearDis = near.distance (pnt.getPosition ());
      double range = rangeRatio * dmin + nearDis;

      ArrayList <Face> faces = new ArrayList <Face> ();
      facesInsideSphere (faces, pnt.getPosition (), range, mesh);
      LinkedList<Face> rFaces = new LinkedList<Face> ();
      rFaces.addAll (faces);
      SRMap.put (pnt, rFaces);
   }
   
   /**
    * retrieve faces in range based on <tt>rangeRatio</tt>; Add
    * point to faces map into <tt>SRMap</tt>; If no face found, 
    * do nothing.
    * @param SRMap source point --->> reference face map 
    * @param pnt source point
    */
   private List <Face> pickFaceInRange (
      Point pnt, PolygonalMesh mesh) {
      Point3d near = new Point3d ();
      BVFeatureQuery.getNearestFaceToPoint (
         near, null, undeformedMesh, pnt.getPosition ());
      double nearDis = near.distance (pnt.getPosition ());
      double range = rangeRatio * dmin + nearDis;

      ArrayList <Face> faces = new ArrayList <Face> ();
      facesInsideSphere (faces, pnt.getPosition (), range, mesh);

      return faces;
   }



   /**
    * Returns a list of all <code>PolygonalMesh</code> face in <tt>mesh</tt> 
    * which inside a sphere.
    * 
    * @param faces returns all faces inside the sphere
    * @param center center of the sphere (world coordinates)
    * @param r sphere radius
    * @param mesh <code>PolygonalMesh</code> contains all faces to be checked;
    */
   public static void facesInsideSphere (
      List<Face> faces, Point3d center, double r, PolygonalMesh mesh) {

      ArrayList<BVNode> nodes = new ArrayList<BVNode> ();
      insideSphere (nodes, center, r, mesh.getBVTree ());
      for (BVNode node : nodes) {
         for (maspack.geometry.Boundable ele : node.getElements ()) {
            faces.add((Face)ele);
         }
      }
   }

   public static void verticesInsideSphere (
      List<Vertex3d> vtxs, Point3d center, double r, PointMesh mesh) {

      ArrayList<BVNode> nodes = new ArrayList<BVNode> ();
      insideSphere (nodes, center, r, mesh.getBVTree ());
      for (BVNode node : nodes) {
         for (maspack.geometry.Boundable ele : node.getElements ()) {
            vtxs.add((Vertex3d)ele);
         }
      }
   }

   public static void linesInsideShpere (
      List<LineSegment> lines, Point3d center, double r, PolylineMesh mesh) {

      ArrayList<BVNode> nodes = new ArrayList<BVNode> ();
      insideSphere (nodes, center, r, mesh.getBVTree ());
      for (BVNode node : nodes) {
         for (maspack.geometry.Boundable ele : node.getElements ()) {
            lines.add((LineSegment)ele);
         }
      }
   }

   /**
    * 
    * @param features
    * @param center
    * @param r
    * @param mesh
    */
   public static void featuresInsideSphere (
      List<Object> features, Point3d center, double r, MeshBase mesh) {
      if (mesh instanceof PolygonalMesh) {
         List<Face> faces = new ArrayList<Face>();
         facesInsideSphere(faces, center, r, (PolygonalMesh)mesh);
         features.addAll (faces);
      }
      else if (mesh instanceof PolylineMesh) {
         List<LineSegment> lines = new ArrayList<LineSegment>();
         linesInsideShpere(lines, center, r, (PolylineMesh)mesh);
         features.addAll (lines);
      }
      else if (mesh instanceof PointMesh) {
         List<Vertex3d> vtxs = new ArrayList<Vertex3d>();
         verticesInsideSphere(vtxs, center, r, (PointMesh)mesh);
         features.addAll (vtxs);
      }
   }

   /**
    * Returns a list of all leaf nodes in this tree which inside a sphere.
    * 
    * @param nodes returns all leaf nodes inside the sphere
    * @param center center of the sphere (world coordinates)
    * @param r sphere radius
    */
   public static void insideSphere (
      List<BVNode> nodes, Point3d center, double r, BVTree tree) {

      RigidTransform3d Xbvw = tree.getBvhToWorld ();
      if (Xbvw != RigidTransform3d.IDENTITY) {
         center = new Point3d (center);
         center.inverseTransform (Xbvw);
      }
      recursivelyInsideSphere (nodes, center, r, tree.getRoot());
   }

   private static void recursivelyInsideSphere (
      List<BVNode> nodes, Point3d origin, double r, BVNode node) {
      if (node.distanceToPoint (origin) < r) {
         if (node.isLeaf()) {
            nodes.add (node);
            return;
         }
         else {
            BVNode child = node.getFirstChild ();
            while (child != null) {
               recursivelyInsideSphere (nodes, origin, r, child);
               child = child.getNext ();
            }
         }
      }
   }

   private int pow = 2;
   private double weightFunc (Point3d pnt1, Point3d pnt2) {
      double w;
      double dis = pnt1.distance (pnt2);
      // inverse distance 
      w = 1.0 / (1 + Math.pow (dis, pow));
      return w;
   }

   @Override
   public void apply (double t0, double t1) {
      updateSlavePoints();
   }

   /**
    * set all slave points to their undeformed position
    */
   protected void resetSlavePoints () {
      Set<Point> pnts = cur2undMap.keySet ();
      for (Point pnt : pnts) {
         Point3d pnt3d = cur2undMap.get (pnt);
         pnt.setPosition (pnt3d);
      }
   }

   public void clearSlavePoints () {
      oldSpRfsMap.clear ();
      currentSpRfsMap.clear ();
      cur2undMap.clear ();
      weightsMap.clear ();
      scaleMap.clear ();
      slavePoints.clear ();
   }
}
